// dualsense_manager.hpp : coordinator for haptic_engine.hpp + adaptive_triggers.hpp
//                         + dualsense_telemetry.hpp
//
// Threads (all owned by the manager, all joined in stop()):
//   Reader : owns the HID handle's lifetime. Blocking hid_read_timeout loop; stamps a host
//            steady_clock time, parses report 0x01, publishes into the TelemetryChannel.
//            Also performs open / reconnect / close, so the handle can never be closed while
//            a read is in flight.
//   Writer : sleeps until a trigger profile changes, coalesces bursts (latest wins), throttles
//            to maxUpdateHz, serializes and hid_write()s. Never closes the handle; on a write
//            failure it flags the device as bad and the reader reconnects.
//
// Handle-sharing protocol (no nested locks anywhere, so no deadlock):
//   * dev_ is only ever MODIFIED by the reader thread, under devMtx_.
//   * The writer holds devMtx_ across each hid_write (short, <= maxUpdateHz per second), so
//     the reader's hid_close() can never overlap a write.
//   * hid_read_timeout runs without any lock (hidapi allows one reader and one writer on the
//     same device); only the reader can close, so it can never close mid-read.
//
// Shutdown order (stop()):  writer first (it releases the triggers while the device is still
//   open) -> then the reader (it closes the device on exit) -> hid_exit().
//
// Threading contract
//   * start() / stop()            : one control thread, not concurrently with each other.
//   * set*Profile(), releaseTriggers(), connected(), stats(), getLatestInput(), drainInput(),
//     haptics().pushFrames()      : any thread (see per-function notes).
//   * Steady state is allocation-free; the reader and writer never touch the heap.
#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>

#include "adaptive_triggers.hpp"
#include "dualsense_telemetry.hpp"
#include "haptic_engine.hpp"

struct hid_device_;  // hidapi's opaque handle; real header is included only in the .cpp

namespace haptic {

struct DualSenseConfig {
    uint16_t vendorId = 0x054C;                                  // Sony
    std::array<uint16_t, 2> productIds{0x0CE6, 0x0DF2};          // DualSense, DualSense Edge (0 = skip)
    double maxUpdateHz = 200.0;                                  // trigger writes; clamped to [10, 250]
    std::chrono::milliseconds reconnectInterval{500};            // retry period while disconnected
    int readTimeoutMs = 10;                                      // clamped to [1, 100]; bounds how long
                                                                 // stop()/unplug detection can take
    size_t telemetryRingFrames = 1024;                           // history ring size; 0 = latest-only
    HapticEngineConfig haptic{};
};

class DualSenseManager {
public:
    struct Stats {
        uint64_t reportsWritten = 0;
        uint64_t writeFailures = 0;
        uint64_t connects = 0;
        uint64_t inputReports = 0;         // parsed and published
        uint64_t inputParseFailures = 0;   // received but not a valid 0x01 USB report
        uint64_t inputDropped = 0;         // history-ring overflows (latest() is unaffected)
    };

    explicit DualSenseManager(const DualSenseConfig& cfg = {});
    ~DualSenseManager();  // calls stop()

    DualSenseManager(const DualSenseManager&) = delete;
    DualSenseManager& operator=(const DualSenseManager&) = delete;
    DualSenseManager(DualSenseManager&&) = delete;
    DualSenseManager& operator=(DualSenseManager&&) = delete;

    // Initialises hidapi, makes one synchronous attempt to open the controller, then starts
    // the reader and writer threads (which keep retrying if the first attempt failed).
    // Returns false if already running, hid_init() fails, or a thread cannot be created
    // (in which case everything started so far has been torn down again). A true return does
    // NOT mean a controller is present; check connected().
    bool start();

    // Releases both trigger effects, joins the writer, then the reader (which closes the
    // device), then calls hid_exit(). Safe to call repeatedly.
    void stop();

    // --- Output: adaptive triggers ---------------------------------------------------
    // Each returns false (and changes nothing) if a parameter is out of range.
    // Pending changes are coalesced: only the newest profile per trigger is ever sent.
    bool setLeftTriggerProfile(const TriggerProfile& profile);
    bool setRightTriggerProfile(const TriggerProfile& profile);
    bool setTriggerProfiles(const TriggerProfile& left, const TriggerProfile& right);
    void releaseTriggers();  // both triggers -> OffProfile

    // --- Input: telemetry ------------------------------------------------------------
    // Newest sample published since the previous call; false if there is nothing new (out is
    // left untouched). Callable from any thread: concurrent callers serialize on a private
    // mutex that the reader thread never touches, so the polling loop stays wait-free. With
    // several consumer threads each sample is delivered to exactly one of them.
    bool getLatestInput(InputState& out);

    // Every sample since the last drain, oldest first (recording/analysis). Same threading
    // rules as getLatestInput(). Returns 0 if telemetryRingFrames was 0.
    size_t drainInput(InputState* dst, size_t maxCount);

    bool connected() const noexcept { return connected_.load(std::memory_order_acquire); }
    Stats stats() const noexcept;

    HapticEngine& haptics() noexcept { return haptics_; }

private:
    void readerLoop();
    void writerLoop();
    bool openDevice();            // reader thread, or start() before threads exist
    void closeDevice() noexcept;  // reader thread, or start() before threads exist
    bool writeReport(const OutputState& state) noexcept;  // writer thread
    void queue(const std::optional<TriggerProfile>& left, const std::optional<TriggerProfile>& right);
    void requestReaderStop();

    const DualSenseConfig cfg_;
    HapticEngine haptics_;
    TelemetryChannel telemetry_;

    // Device handle: modified only by the reader (under devMtx_), used by both threads.
    std::mutex devMtx_;
    struct hid_device_* dev_ = nullptr;
    std::atomic<bool> connected_{false};
    std::atomic<bool> deviceError_{false};  // writer -> reader: "this handle is bad, reopen it"

    // Writer-thread-only.
    OutputState applied_{};  // last state the controller confirmed; replayed after reconnect
    OutputReport report_{};

    // Consumers of telemetry_ (UI side). The reader thread never takes this.
    std::mutex consumerMtx_;

    std::thread readerThread_;
    std::thread writerThread_;

    std::mutex mtx_;                      // guards pending_, dirty_, connectEpoch_, running_
    std::condition_variable cv_;          // wakes the writer (setters, connect, stop)
    std::condition_variable readerCv_;    // wakes the reader's reconnect sleep (stop only)
    OutputState pending_{};
    bool dirty_ = false;
    uint32_t connectEpoch_ = 0;           // bumped by every successful open
    bool running_ = false;
    std::atomic<bool> stop_{false};        // ask the writer to finish (stored under mtx_)
    std::atomic<bool> readerStop_{false};  // ask the reader to finish (set only after writer joined)

    std::atomic<uint64_t> reportsWritten_{0};
    std::atomic<uint64_t> writeFailures_{0};
    std::atomic<uint64_t> connects_{0};
    std::atomic<uint64_t> inputReports_{0};
    std::atomic<uint64_t> inputParseFailures_{0};
};

}  // namespace haptic
