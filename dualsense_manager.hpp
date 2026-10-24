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

struct hid_device;

namespace haptic {

struct HapticEngineConfig {
    double sampleRate = 48000.0;
    uint32_t outputChannels = 4;   
    uint32_t leftActuator = 2;     
    uint32_t rightActuator = 3;    
};

class HapticEngine {
public:
    HapticEngine() = default;
    explicit HapticEngine(const HapticEngineConfig& cfg) : cfg_(cfg) {}
    void setDrive(float linearGain) noexcept { drive_ = linearGain; }
    size_t pushFrames(const float*, size_t frames) noexcept { return frames; }
    void render(float* outBuffer, uint32_t frames) noexcept {
        std::fill_n(outBuffer, frames * cfg_.outputChannels, 0.0f);
    }
private:
    HapticEngineConfig cfg_;
    float drive_ = 1.0f;
};

struct DualSenseConfig {
    uint16_t vendorId = 0x054C;                                  
    std::array<uint16_t, 2> productIds{0x0CE6, 0x0DF2};          
    double maxUpdateHz = 200.0;                                  
    std::chrono::milliseconds reconnectInterval{500};            
    int readTimeoutMs = 10;                                      
    size_t telemetryRingFrames = 1024;                           
    HapticEngineConfig haptic_cfg; 
};

class DualSenseManager {
public:
    struct Stats {
        uint64_t reportsWritten = 0;
        uint64_t writeFailures = 0;
        uint64_t connects = 0;
        uint64_t inputReports = 0;         
        uint64_t inputParseFailures = 0;   
        uint64_t inputDropped = 0;         
    };

    explicit DualSenseManager(const DualSenseConfig& cfg = {});
    ~DualSenseManager();  

    DualSenseManager(const DualSenseManager&) = delete;
    DualSenseManager& operator=(const DualSenseManager&) = delete;
    DualSenseManager(DualSenseManager&&) = delete;
    DualSenseManager& operator=(DualSenseManager&&) = delete;

    bool start();
    void stop();

    bool setLeftTriggerProfile(const TriggerProfile& profile);
    bool setRightTriggerProfile(const TriggerProfile& profile);
    bool setTriggerProfiles(const TriggerProfile& left, const TriggerProfile& right);
    void releaseTriggers();  

    bool getLatestInput(InputState& out);
    size_t drainInput(InputState* dst, size_t maxCount);

    bool connected() const noexcept { return connected_.load(std::memory_order_acquire); }
    Stats stats() const noexcept;

    HapticEngine& haptics() noexcept { return haptics_; }

private:
    void readerLoop();
    void writerLoop();
    bool openDevice();            
    void closeDevice() noexcept;  
    bool writeReport(const OutputState& state) noexcept;  
    void queue(const std::optional<TriggerProfile>& left, const std::optional<TriggerProfile>& right);
    void requestReaderStop();

    const DualSenseConfig cfg_;
    HapticEngine haptics_;
    TelemetryChannel telemetry_;

    std::mutex devMtx_;
    struct hid_device* dev_ = nullptr; 
    std::atomic<bool> connected_{false};
    std::atomic<bool> deviceError_{false};  

    OutputState applied_{};  
    OutputReport report_{};

    std::mutex consumerMtx_;

    std::thread readerThread_;
    std::thread writerThread_;

    std::mutex mtx_;                      
    std::condition_variable cv_;          
    std::condition_variable readerCv_;    
    OutputState pending_{};
    bool dirty_ = false;
    uint32_t connectEpoch_ = 0;           
    bool running_ = false;
    std::atomic<bool> stop_{false};        
    std::atomic<bool> readerStop_{false};  

    std::atomic<uint64_t> reportsWritten_{0};
    std::atomic<uint64_t> writeFailures_{0};
    std::atomic<uint64_t> connects_{0};
    std::atomic<uint64_t> inputReports_{0};
    std::atomic<uint64_t> inputParseFailures_{0};
};

}  // namespace haptic
