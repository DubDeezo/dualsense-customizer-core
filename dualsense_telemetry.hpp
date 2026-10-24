// dualsense_telemetry.hpp : inbound telemetry parser and mailboxes
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace haptic {

constexpr uint8_t kInputReportId = 0x01;
constexpr size_t kUsbInputReportSize = 64;  
constexpr int kTouchpadMaxX = 1919;  
constexpr int kTouchpadMaxY = 1079;  

namespace btn {
constexpr uint32_t Square = 1u << 0;
constexpr uint32_t Cross = 1u << 1;
constexpr uint32_t Circle = 1u << 2;
constexpr uint32_t Triangle = 1u << 3;
constexpr uint32_t L1 = 1u << 4;
constexpr uint32_t R1 = 1u << 5;
constexpr uint32_t L2 = 1u << 6;  
constexpr uint32_t R2 = 1u << 7;
constexpr uint32_t Create = 1u << 8;
constexpr uint32_t Options = 1u << 9;
constexpr uint32_t L3 = 1u << 10;
constexpr uint32_t R3 = 1u << 11;
constexpr uint32_t Home = 1u << 12;
constexpr uint32_t Touchpad = 1u << 13;  
constexpr uint32_t Mute = 1u << 14;
constexpr uint32_t DpadUp = 1u << 15;
constexpr uint32_t DpadRight = 1u << 16;
constexpr uint32_t DpadDown = 1u << 17;
constexpr uint32_t DpadLeft = 1u << 18;
}  // namespace btn

struct TouchPoint {
    uint16_t x = 0;      
    uint16_t y = 0;      
    uint8_t id = 0;      
    bool active = false; 
};

struct InputState {
    uint8_t leftStickX = 128, leftStickY = 128, rightStickX = 128, rightStickY = 128;
    uint8_t l2 = 0, r2 = 0;  
    uint8_t dpad = 8;        
    uint8_t sequence = 0;    
    uint32_t buttons = 0;    
    int16_t gyro[3] = {0, 0, 0};   
    int16_t accel[3] = {0, 0, 0};  
    uint32_t sensorTimestamp = 0;  
    int8_t temperature = 0;
    TouchPoint touch[2];
    uint8_t leftTriggerStop = 0, rightTriggerStop = 0;      
    uint8_t leftTriggerStatus = 0, rightTriggerStatus = 0;  
    uint8_t batteryLevel = 0;  
    uint8_t batteryState = 0;  
    uint64_t hostTimeNs = 0;   
};

namespace detail {
inline uint16_t le16(const uint8_t* p) noexcept { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
inline int16_t leS16(const uint8_t* p) noexcept { return static_cast<int16_t>(le16(p)); }
inline uint32_t le32(const uint8_t* p) noexcept {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
constexpr std::array<uint32_t, 9> kDpadBits = {
    btn::DpadUp, btn::DpadUp | btn::DpadRight, btn::DpadRight,
    btn::DpadDown | btn::DpadRight, btn::DpadDown, btn::DpadDown | btn::DpadLeft,
    btn::DpadLeft, btn::DpadUp | btn::DpadLeft, 0u
};
inline TouchPoint decodeTouch(const uint8_t* p) noexcept {
    TouchPoint t;
    t.active = (p[0] & 0x80) == 0;
    t.id = static_cast<uint8_t>(p[0] & 0x7F);
    t.x = static_cast<uint16_t>(p[1] | ((p[2] & 0x0F) << 8));
    t.y = static_cast<uint16_t>((p[2] >> 4) | (p[3] << 4));
    return t;
}
}  // namespace detail

inline bool parseInputReport(const uint8_t* buffer, size_t size, InputState& out) noexcept {
    if (buffer == nullptr || size < kUsbInputReportSize || buffer[0] != kInputReportId) return false;
    const uint8_t* s = buffer + 1;  
    out.leftStickX = s[0]; out.leftStickY = s[1]; out.rightStickX = s[2]; out.rightStickY = s[3];
    out.l2 = s[4]; out.r2 = s[5]; out.sequence = s[6];
    uint8_t hat = static_cast<uint8_t>(s[7] & 0x0F);
    if (hat > 8) hat = 8;
    out.dpad = hat;
    uint32_t m = detail::kDpadBits[hat];
    if (s[7] & 0x10) m |= btn::Square;
    if (s[7] & 0x20) m |= btn::Cross;
    if (s[7] & 0x40) m |= btn::Circle;
    if (s[7] & 0x80) m |= btn::Triangle;
    if (s[8] & 0x01) m |= btn::L1;
    if (s[8] & 0x02) m |= btn::R1;
    if (s[8] & 0x04) m |= btn::L2;
    if (s[8] & 0x08) m |= btn::R2;
    if (s[8] & 0x10) m |= btn::Create;
    if (s[8] & 0x20) m |= btn::Options;
    if (s[8] & 0x40) m |= btn::L3;
    if (s[8] & 0x80) m |= btn::R3;
    if (s[9] & 0x01) m |= btn::Home;
    if (s[9] & 0x02) m |= btn::Touchpad;
    if (s[9] & 0x04) m |= btn::Mute;
    out.buttons = m;
    for (int i = 0; i < 3; ++i) {
        out.gyro[i] = detail::leS16(s + 15 + 2 * i);
        out.accel[i] = detail::leS16(s + 21 + 2 * i);
    }
    out.sensorTimestamp = detail::le32(s + 27);
    out.temperature = static_cast<int8_t>(s[31]);
    out.touch[0] = detail::decodeTouch(s + 32);
    out.touch[1] = detail::decodeTouch(s + 36);
    out.rightTriggerStop = static_cast<uint8_t>(s[41] & 0x0F);
    out.rightTriggerStatus = static_cast<uint8_t>(s[41] >> 4);
    out.leftTriggerStop = static_cast<uint8_t>(s[42] & 0x0F);
    out.leftTriggerStatus = static_cast<uint8_t>(s[42] >> 4);
    out.batteryLevel = static_cast<uint8_t>(s[52] & 0x0F);
    out.batteryState = static_cast<uint8_t>(s[52] >> 4);
    return true;
}

constexpr bool isDown(const InputState& s, uint32_t buttonBits) noexcept { return (s.buttons & buttonBits) == buttonBits; }
constexpr float stickToFloat(uint8_t v) noexcept { return (static_cast<float>(v) - 127.5f) / 127.5f; }  
constexpr float triggerToFloat(uint8_t v) noexcept { return static_cast<float>(v) / 255.0f; }          

template <typename T>
class TripleBuffer {
public:
    void publish(const T& value) noexcept {
        slots_[back_] = value;
        uint32_t prev = middle_.exchange(back_ | kFresh, std::memory_order_acq_rel);
        back_ = prev & kIndexMask;
    }
    bool consume(T& out) noexcept {
        if ((middle_.load(std::memory_order_acquire) & kFresh) == 0) return false;
        uint32_t prev = middle_.exchange(front_, std::memory_order_acq_rel);
        front_ = prev & kIndexMask;
        out = slots_[front_];
        return true;
    }
private:
    static constexpr uint32_t kFresh = 4u;
    static constexpr uint32_t kIndexMask = 3u;
    T slots_[3]{};
    uint32_t back_ = 0;   
    uint32_t front_ = 2;  
    alignas(64) std::atomic<uint32_t> middle_{1};  
};

class TelemetryChannel {
public:
    explicit TelemetryChannel(size_t) {}
    void publish(const InputState& s) noexcept { latest_.publish(s); }
    bool latest(InputState& out) noexcept { return latest_.consume(out); }
    size_t drain(InputState*, size_t) noexcept { return 0; }
    uint64_t dropped() const noexcept { return 0; }
private:
    TripleBuffer<InputState> latest_;
};

}  // namespace haptic
