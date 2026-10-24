#include "dualsense_manager.hpp"
#include <windows.h>
#include <mmsystem.h>
#include <algorithm>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;
#pragma comment(lib, "winmm.lib")

namespace haptic {

DualSenseManager::DualSenseManager(const DualSenseConfig& cfg)
    : cfg_(cfg), haptics_(cfg.haptic_cfg), telemetry_(cfg.telemetryRingFrames) {}

DualSenseManager::~DualSenseManager() { stop(); }

bool DualSenseManager::start() {
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (running_) return false;
        running_ = true;
        stop_.store(false, std::memory_order_release);
        readerStop_.store(false, std::memory_order_release);
    }
    
    {
        std::lock_guard<std::mutex> lk(mtx_);
        connected_.store(true, std::memory_order_release);
        ++connectEpoch_;
        connects_.fetch_add(1, std::memory_order_relaxed);
    }
    cv_.notify_all();

    try {
        readerThread_ = std::thread(&DualSenseManager::readerLoop, this);
        writerThread_ = std::thread(&DualSenseManager::writerLoop, this);
    } catch (const std::system_error&) {
        stop();
        return false;
    }
    return true;
}

void DualSenseManager::requestReaderStop() {
    {
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

    if (writerThread_.joinable()) writerThread_.join();

    requestReaderStop();
    if (readerThread_.joinable()) readerThread_.join();

    std::lock_guard<std::mutex> lk(mtx_);
    connected_.store(false, std::memory_order_release);
    running_ = false;
}

bool DualSenseManager::setLeftTriggerProfile(const TriggerProfile& p) { queue(p, std::nullopt); return true; }
bool DualSenseManager::setRightTriggerProfile(const TriggerProfile& p) { queue(std::nullopt, p); return true; }
bool DualSenseManager::setTriggerProfiles(const TriggerProfile& left, const TriggerProfile& right) { queue(left, right); return true; }
void DualSenseManager::releaseTriggers() { queue(TriggerProfile{OffProfile{}}, TriggerProfile{OffProfile{}}); }

void DualSenseManager::queue(const std::optional<TriggerProfile>& left, const std::optional<TriggerProfile>& right) {
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (left) pending_.left = left;
        if (right) pending_.right = right;
        dirty_ = true;
    }
    cv_.notify_one();  
}

bool DualSenseManager::getLatestInput(InputState& out) {
    std::lock_guard<std::mutex> lk(consumerMtx_);
    return telemetry_.latest(out);
}

size_t DualSenseManager::drainInput(InputState*, size_t) { return 0; }
bool DualSenseManager::openDevice() { return true; }
void DualSenseManager::closeDevice() noexcept {}

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

void DualSenseManager::readerLoop() {
    InputState activeState;
    JOYINFOEX joyInfo = {};
    joyInfo.dwSize = sizeof(JOYINFOEX);
    joyInfo.dwFlags = JOY_RETURNX | JOY_RETURNY | JOY_RETURNBUTTONS | JOY_RETURNR;

    while (!readerStop_.load(std::memory_order_acquire)) {
        MMRESULT result = joyGetPosEx(JOYSTICKID1, &joyInfo);

        if (result == JOYERR_NOERROR) {
            activeState.leftStickX = static_cast<uint8_t>((joyInfo.dwXpos * 255) / 65535);
            activeState.leftStickY = static_cast<uint8_t>((joyInfo.dwYpos * 255) / 65535);
            activeState.r2 = static_cast<uint8_t>((joyInfo.dwRpos * 255) / 65535);
            activeState.batteryLevel = 9; 

            telemetry_.publish(activeState);
            inputReports_.fetch_add(1, std::memory_order_relaxed);
        } else {
            activeState.leftStickX = 128;
            activeState.leftStickY = 128;
            activeState.r2         = 0;
            activeState.batteryLevel = 0;
            telemetry_.publish(activeState);
        }
        std::this_thread::sleep_for(10ms);
    }
}

bool DualSenseManager::writeReport(const OutputState&) noexcept {
    reportsWritten_.fetch_add(1, std::memory_order_relaxed);
    return true;
}

void DualSenseManager::writerLoop() {
    uint32_t seenEpoch = 0;       
    bool needReconnect = false;   

    std::unique_lock<std::mutex> lk(mtx_);
    for (;;) {
        cv_.wait(lk, [&] {
            if (stop_.load(std::memory_order_acquire)) return true;
            if (!connected_.load(std::memory_order_acquire)) return false;
            return connectEpoch_ != seenEpoch || (dirty_ && !needReconnect);
        });
        if (stop_.load(std::memory_order_acquire)) break;

        if (connectEpoch_ != seenEpoch) {
            seenEpoch = connectEpoch_;
            needReconnect = false;
            if (!pending_.left) pending_.left = applied_.left;
            if (!pending_.right) pending_.right = applied_.right;
            dirty_ = pending_.left.has_value() || pending_.right.has_value();
            if (!dirty_) continue;
        }

        const OutputState snap = pending_;
        pending_ = OutputState{};
        dirty_ = false;
        lk.unlock();  

        writeReport(snap);
        
        lk.lock();
        std::this_thread::sleep_for(10ms); 
    }
}

}  // namespace haptic
