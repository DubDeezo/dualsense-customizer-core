// dualsense_manager.cpp
// Build: link against hidapi (hidapi::hidapi, or hidapi-hidraw / hidapi-libusb on Linux) and
// the platform thread library (-pthread).
// Some distros install the header as <hidapi.h> instead of <hidapi/hidapi.h>.
#include "dualsense_manager.hpp"

#include <hidapi/hidapi.h>

#include <algorithm>
#include <system_error>

namespace haptic {

DualSenseManager::DualSenseManager(const DualSenseConfig& cfg)
    : cfg_(cfg), haptics_(cfg.haptic), telemetry_(cfg.telemetryRingFrames) {}

DualSenseManager::~DualSenseManager() { stop(); }

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
bool DualSenseManager::start() {
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (running_) return false;
        running_ = true;
        stop_.store(false, std::memory_order_release);
        readerStop_.store(false, std::memory_order_release);
    }
    if (hid_init() != 0) {
        std::lock_guard<std::mutex> lk(mtx_);
        running_ = false;
        return false;
    }
    openDevice();  // immediate feedback via connected(); the reader retries if this fails

    bool readerStarted = false;
    try {
        readerThread_ = std::thread(&DualSenseManager::readerLoop, this);
        readerStarted = true;
        writerThread_ = std::thread(&DualSenseManager::writerLoop, this);
    } catch (const std::system_error&) {
        // Roll back exactly what was started; nothing is left running or open.
        if (readerStarted) {
            requestReaderStop();
            readerThread_.join();  // reader closes the device on exit
        } else {
            closeDevice();
        }
        hid_exit();
        std::lock_guard<std::mutex> lk(mtx_);
        running_ = false;
        return false;
    }
    return true;
}

void DualSenseManager::requestReaderStop() {
    {
        // Store under mtx_ so a reader about to sleep in wait_for cannot miss the wakeup.
        std::lock_guard<std::mutex> lk(mtx_);
        readerStop_.store(true, std::memory_order_release);
    }
    readerCv_.notify_all();
}

void DualSenseManager::stop() {
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!running_) return;
        stop_.store(true, std::memory_order_release);
    }
    cv_.notify_all();

    // 1. Writer first: its last act is to release the triggers, which needs the device
    //    still open - and the reader is what keeps it open.
    if (writerThread_.joinable()) writerThread_.join();

    // 2. Now the reader: it leaves its loop, closes the device and exits.
    requestReaderStop();
    if (readerThread_.joinable()) readerThread_.join();

    // 3. No thread can touch hidapi any more.
    hid_exit();
    std::lock_guard<std::mutex> lk(mtx_);
    running_ = false;
}

bool DualSenseManager::openDevice() {
    for (const uint16_t pid : cfg_.productIds) {
        if (pid == 0) continue;
        hid_device* d = hid_open(cfg_.vendorId, pid, nullptr);
        if (!d) continue;
        {
            std::lock_guard<std::mutex> dl(devMtx_);
            dev_ = d;
        }
        deviceError_.store(false, std::memory_order_release);
        {
            // Publish "connected" and the new epoch together so the writer cannot miss it.
            std::lock_guard<std::mutex> lk(mtx_);
            connected_.store(true, std::memory_order_release);
            ++connectEpoch_;
        }
        cv_.notify_all();
        connects_.fetch_add(1, std::memory_order_relaxed);
        return true;
    }
    return false;
}

void DualSenseManager::closeDevice() noexcept {
    connected_.store(false, std::memory_order_release);
    {
        std::lock_guard<std::mutex> dl(devMtx_);  // waits out any in-flight hid_write
        if (dev_) {
            hid_close(dev_);
            dev_ = nullptr;
        }
    }
    deviceError_.store(false, std::memory_order_release);
}

// ---------------------------------------------------------------------------
// Application interface
// ---------------------------------------------------------------------------
void DualSenseManager::queue(const std::optional<TriggerProfile>& left,
                             const std::optional<TriggerProfile>& right) {
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (left) pending_.left = left;
        if (right) pending_.right = right;
        dirty_ = true;
    }
    cv_.notify_one();  // only the writer ever waits on cv_
}

bool DualSenseManager::setLeftTriggerProfile(const TriggerProfile& p) {
    TriggerBlock scratch;
    if (!encodeTriggerProfile(p, scratch)) return false;
    queue(p, std::nullopt);
    return true;
}

bool DualSenseManager::setRightTriggerProfile(const TriggerProfile& p) {
    TriggerBlock scratch;
    if (!encodeTriggerProfile(p, scratch)) return false;
    queue(std::nullopt, p);
    return true;
}

bool DualSenseManager::setTriggerProfiles(const TriggerProfile& left, const TriggerProfile& right) {
    TriggerBlock scratch;
    if (!encodeTriggerProfile(left, scratch) || !encodeTriggerProfile(right, scratch)) return false;
    queue(left, right);  // one lock, one wake: both triggers land in the same report
    return true;
}

void DualSenseManager::releaseTriggers() {
    queue(TriggerProfile{OffProfile{}}, TriggerProfile{OffProfile{}});
}

bool DualSenseManager::getLatestInput(InputState& out) {
    std::lock_guard<std::mutex> lk(consumerMtx_);
    return telemetry_.latest(out);
}

size_t DualSenseManager::drainInput(InputState* dst, size_t maxCount) {
    std::lock_guard<std::mutex> lk(consumerMtx_);
    return telemetry_.drain(dst, maxCount);
}

DualSenseManager::Stats DualSenseManager::stats() const noexcept {
    Stats s;
    s.reportsWritten = reportsWritten_.load(std::memory_order_relaxed);
    s.writeFailures = writeFailures_.load(std::memory_order_relaxed);
    s.connects = connects_.load(std::memory_order_relaxed);
    s.inputReports = inputReports_.load(std::memory_order_relaxed);
    s.inputParseFailures = inputParseFailures_.load(std::memory_order_relaxed);
    s.inputDropped = telemetry_.dropped();
    return s;
}

// ---------------------------------------------------------------------------
// Reader thread: owns open/reconnect/close and the blocking read loop
// ---------------------------------------------------------------------------
void DualSenseManager::readerLoop() {
    using Clock = std::chrono::steady_clock;
    const int timeoutMs = std::clamp(cfg_.readTimeoutMs, 1, 100);

    std::array<uint8_t, 128> buf{};  // larger than the 64-byte report: never truncates
    InputState state;                // reused every iteration; only the parser and stamp write it

    while (!readerStop_.load(std::memory_order_acquire)) {
        // ---- Disconnected: retry periodically (interruptible by stop) ----
        if (!connected_.load(std::memory_order_acquire)) {
            if (openDevice()) continue;
            std::unique_lock<std::mutex> lk(mtx_);
            readerCv_.wait_for(lk, cfg_.reconnectInterval,
                               [this] { return readerStop_.load(std::memory_order_acquire); });
            continue;
        }

        // ---- The writer saw a failed write: this handle is bad, reopen it ----
        if (deviceError_.load(std::memory_order_acquire)) {
            closeDevice();
            continue;
        }

        // ---- Blocking read. dev_ is only modified by this thread, so no lock is needed ----
        const int n = hid_read_timeout(dev_, buf.data(), buf.size(), timeoutMs);
        if (n < 0) {  // unplugged / error
            closeDevice();
            continue;
        }
        if (n == 0) continue;  // timeout: loop back to re-check stop and the error flag

        // Stamp immediately after the read returns, before any parsing work.
        const uint64_t nowNs = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count());

        if (parseInputReport(buf.data(), static_cast<size_t>(n), state)) {
            state.hostTimeNs = nowNs;
            telemetry_.publish(state);  // wait-free; never blocks the polling thread
            inputReports_.fetch_add(1, std::memory_order_relaxed);
        } else {
            inputParseFailures_.fetch_add(1, std::memory_order_relaxed);
        }
    }

    closeDevice();  // the one place the handle is released at shutdown
}

// ---------------------------------------------------------------------------
// Writer thread
// ---------------------------------------------------------------------------
bool DualSenseManager::writeReport(const OutputState& state) noexcept {
    if (serializeOutputReport(state, report_) != SerializeStatus::Ok) return true;  // unreachable: setters validate

    std::lock_guard<std::mutex> dl(devMtx_);  // keeps the reader from closing mid-write
    if (!dev_) return false;                  // reader already dropped the handle
    const int n = hid_write(dev_, report_.data(), report_.size());
    if (n < 0) {
        writeFailures_.fetch_add(1, std::memory_order_relaxed);
        deviceError_.store(true, std::memory_order_release);  // reader will close + reopen
        return false;
    }
    reportsWritten_.fetch_add(1, std::memory_order_relaxed);
    return true;
}

void DualSenseManager::writerLoop() {
    using Clock = std::chrono::steady_clock;
    const double hz = std::clamp(cfg_.maxUpdateHz, 10.0, 250.0);
    const auto minInterval =
        std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / hz));
    Clock::time_point lastWrite{};  // epoch => the first write is never delayed

    uint32_t seenEpoch = 0;       // last connection we replayed state onto
    bool needReconnect = false;   // a write failed; hold off until the reader reopens the device

    std::unique_lock<std::mutex> lk(mtx_);
    for (;;) {
        cv_.wait(lk, [&] {
            if (stop_.load(std::memory_order_acquire)) return true;
            if (!connected_.load(std::memory_order_acquire)) return false;
            return connectEpoch_ != seenEpoch || (dirty_ && !needReconnect);
        });
        if (stop_.load(std::memory_order_acquire)) break;

        // ---- (Re)connected: replay what the controller last had under any newer change ----
        if (connectEpoch_ != seenEpoch) {
            seenEpoch = connectEpoch_;
            needReconnect = false;
            if (!pending_.left) pending_.left = applied_.left;
            if (!pending_.right) pending_.right = applied_.right;
            dirty_ = pending_.left.has_value() || pending_.right.has_value();
            if (!dirty_) continue;
        }

        // ---- Throttle. Setters keep overwriting pending_ meanwhile, so bursts coalesce ----
        const auto due = lastWrite + minInterval;
        if (Clock::now() < due &&
            cv_.wait_until(lk, due, [this] { return stop_.load(std::memory_order_acquire); }))
            break;

        const OutputState snap = pending_;
        pending_ = OutputState{};
        dirty_ = false;
        lk.unlock();  // never hold mtx_ across a (potentially blocking) hid_write

        const bool ok = writeReport(snap);
        lastWrite = Clock::now();
        if (ok) {
            if (snap.left) applied_.left = snap.left;
            if (snap.right) applied_.right = snap.right;
        }

        lk.lock();
        if (!ok) {  // keep the unsent change, but never overwrite a newer one
            if (!pending_.left) pending_.left = snap.left;
            if (!pending_.right) pending_.right = snap.right;
            dirty_ = pending_.left.has_value() || pending_.right.has_value();
            needReconnect = true;
        }
    }
    lk.unlock();

    // ---- Shutdown: leave the controller with neutral triggers. The reader still holds the
    //      device open (stop() joins this thread first), so this write is safe. ----
    if (connected_.load(std::memory_order_acquire)) {
        OutputState off;
        off.left = OffProfile{};
        off.right = OffProfile{};
        writeReport(off);
    }
    applied_ = OutputState{};  // a later start() begins from a clean slate
}

}  // namespace haptic
