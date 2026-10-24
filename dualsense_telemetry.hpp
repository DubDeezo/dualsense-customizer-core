// dualsense_telemetry.hpp : header-only, C++17, no dependencies beyond haptic_engine.hpp.
//
// Inbound telemetry for a DualSense-style controller over USB:
//   * InputState          : small, trivially copyable snapshot of everything useful.
//   * parseInputReport()  : pure, allocation-free decoder for the 64-byte USB report 0x01.
//   * TripleBuffer<T>     : wait-free "latest value" mailbox (UI always sees the newest sample).
//   * TelemetryChannel    : TripleBuffer + SpscRingBuffer behind one publish() call, so the
//                           polling thread never blocks and the consumer picks its style
//                           (latest snapshot, or every sample for recording/analysis).
//
// Report 0x01 over USB (offsets include the report ID at byte 0; layout follows the Linux
// hid-playstation driver and the community wiki):
//   [ 1..4 ]  left stick X/Y, right stick X/Y (0..255, ~128 centre)
//   [ 5..6 ]  L2 / R2 analog (0..255)
//   [ 7 ]     sequence counter
//   [ 8 ]     d-pad (low nibble, 0=N clockwise..7=NW, 8=none) | Square/Cross/Circle/Triangle
//   [ 9 ]     L1 R1 L2 R2 Create Options L3 R3
//   [10]      Home, Touchpad click, Mute, Edge function/paddle buttons
//   [16..27]  gyro x3, accel x3 (int16 little endian, interleaved as reported)
//   [28..31]  sensor timestamp (uint32 LE)
//   [32]      temperature (int8)
//   [33..40]  two touch points, 4 bytes each: id/contact, then packed 12-bit X and Y
//   [42..43]  trigger stop location (low nibble) / status (high nibble), right then left
//   [53]      battery level (low nibble 0..10) / charge state (high nibble)
//
// Threading contract:
//   * parseInputReport(): pure; any thread.
//   * TelemetryChannel::publish(): ONE producer thread (the HID reader loop).
//   * TelemetryChannel::latest()/drain(): ONE consumer thread (UI/app).
//   Nothing here allocates after construction; nothing locks.
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "haptic_engine.hpp"  // SpscRingBuffer

namespace haptic {

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
constexpr uint8_t kInputReportId = 0x01;
constexpr size_t kUsbInputReportSize = 64;  // report ID + 63 data bytes

constexpr int kTouchpadMaxX = 1919;  // 12-bit field, but the sensor only reports 0..1919
constexpr int kTouchpadMaxY = 1079;  //                                          and 0..1079

// Nominal sensor scales (the normalisation targets used by the Linux driver after applying
// the factory calibration from feature report 0x05). Raw, uncalibrated counts will differ.
constexpr float kGyroCountsPerDegPerSec = 1024.0f;
constexpr float kAccelCountsPerG = 8192.0f;

// Button bits inside InputState::buttons. The d-pad is also exposed as four digital bits so
// callers can ignore the raw 0..8 hat value.
namespace btn {
constexpr uint32_t Square = 1u << 0;
constexpr uint32_t Cross = 1u << 1;
constexpr uint32_t Circle = 1u << 2;
constexpr uint32_t Triangle = 1u << 3;
constexpr uint32_t L1 = 1u << 4;
constexpr uint32_t R1 = 1u << 5;
constexpr uint32_t L2 = 1u << 6;  // digital click point of the trigger
constexpr uint32_t R2 = 1u << 7;
constexpr uint32_t Create = 1u << 8;
constexpr uint32_t Options = 1u << 9;
constexpr uint32_t L3 = 1u << 10;
constexpr uint32_t R3 = 1u << 11;
constexpr uint32_t Home = 1u << 12;
constexpr uint32_t Touchpad = 1u << 13;  // pad click, not touch
constexpr uint32_t Mute = 1u << 14;
constexpr uint32_t DpadUp = 1u << 15;
constexpr uint32_t DpadRight = 1u << 16;
constexpr uint32_t DpadDown = 1u << 17;
constexpr uint32_t DpadLeft = 1u << 18;
constexpr uint32_t EdgeLeftFn = 1u << 19;  // DualSense Edge only
constexpr uint32_t EdgeRightFn = 1u << 20;
constexpr uint32_t EdgeLeftPaddle = 1u << 21;
constexpr uint32_t EdgeRightPaddle = 1u << 22;
}  // namespace btn

// ---------------------------------------------------------------------------
// Snapshot types
// ---------------------------------------------------------------------------
struct TouchPoint {
    uint16_t x = 0;      // 0..1919 (raw 12-bit value; stale while !active)
    uint16_t y = 0;      // 0..1079
    uint8_t id = 0;      // 7-bit contact id, increments per new touch
    bool active = false; // finger currently on the pad
};

struct InputState {
    uint8_t leftStickX = 128, leftStickY = 128, rightStickX = 128, rightStickY = 128;
    uint8_t l2 = 0, r2 = 0;  // analog trigger travel, 0..255
    uint8_t dpad = 8;        // raw hat: 0=N, 1=NE ... 7=NW, 8=released
    uint8_t sequence = 0;    // report counter from the controller (wraps at 256)
    uint32_t buttons = 0;    // btn:: bits

    int16_t gyro[3] = {0, 0, 0};   // as reported; the community docs disagree on axis labels
    int16_t accel[3] = {0, 0, 0};  // as reported
    uint32_t sensorTimestamp = 0;  // controller clock, ~0.33 us per tick
    int8_t temperature = 0;

    TouchPoint touch[2];

    // Adaptive trigger feedback (useful for confirming effects from adaptive_triggers.hpp).
    uint8_t leftTriggerStop = 0, rightTriggerStop = 0;      // 0..9 physical stop zone
    uint8_t leftTriggerStatus = 0, rightTriggerStatus = 0;  // effect-specific state nibble

    uint8_t batteryLevel = 0;  // 0..10
    uint8_t batteryState = 0;  // 0 discharging, 1 charging, 2 full (others = fault)

    uint64_t hostTimeNs = 0;   // stamped by the reader thread; parseInputReport() never writes it
};
static_assert(std::is_trivially_copyable<InputState>::value, "InputState must stay trivially copyable");

enum class ParseStatus { Ok, WrongReportId, TooShort };

// ---------------------------------------------------------------------------
// Parsing
// ---------------------------------------------------------------------------
namespace detail {

inline uint16_t le16(const uint8_t* p) noexcept {
    return static_cast<uint16_t>(p[0] | (p[1] << 8));
}
inline int16_t leS16(const uint8_t* p) noexcept { return static_cast<int16_t>(le16(p)); }
inline uint32_t le32(const uint8_t* p) noexcept {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

// d-pad hat value (0..8) -> digital bits
constexpr std::array<uint32_t, 9> kDpadBits = {
    btn::DpadUp,                  // 0 N
    btn::DpadUp | btn::DpadRight, // 1 NE
    btn::DpadRight,               // 2 E
    btn::DpadDown | btn::DpadRight, // 3 SE
    btn::DpadDown,                // 4 S
    btn::DpadDown | btn::DpadLeft,  // 5 SW
    btn::DpadLeft,                // 6 W
    btn::DpadUp | btn::DpadLeft,  // 7 NW
    0u,                           // 8 released
};

// 4-byte touch record: [0] bit7 = NOT touching, bits0..6 = id;
// [1..3] little-endian 24 bits: X = low 12, Y = high 12.
inline TouchPoint decodeTouch(const uint8_t* p) noexcept {
    TouchPoint t;
    t.active = (p[0] & 0x80) == 0;
    t.id = static_cast<uint8_t>(p[0] & 0x7F);
    t.x = static_cast<uint16_t>(p[1] | ((p[2] & 0x0F) << 8));
    t.y = static_cast<uint16_t>((p[2] >> 4) | (p[3] << 4));
    return t;
}

}  // namespace detail

// Decodes the USB 0x01 input report. On failure `out` is not modified.
// (The short 10-byte Bluetooth 0x01 report has a different layout and is not handled.)
inline ParseStatus parseInputReportStatus(const uint8_t* buffer, size_t size, InputState& out) noexcept {
    if (buffer == nullptr || size < 1 || buffer[0] != kInputReportId) return ParseStatus::WrongReportId;
    if (size < kUsbInputReportSize) return ParseStatus::TooShort;

    const uint8_t* s = buffer + 1;  // state block; offsets below are relative to it

    out.leftStickX = s[0];
    out.leftStickY = s[1];
    out.rightStickX = s[2];
    out.rightStickY = s[3];
    out.l2 = s[4];
    out.r2 = s[5];
    out.sequence = s[6];

    const uint8_t b7 = s[7], b8 = s[8], b9 = s[9];
    uint8_t hat = static_cast<uint8_t>(b7 & 0x0F);
    if (hat > 8) hat = 8;
    out.dpad = hat;

    uint32_t m = detail::kDpadBits[hat];
    if (b7 & 0x10) m |= btn::Square;
    if (b7 & 0x20) m |= btn::Cross;
    if (b7 & 0x40) m |= btn::Circle;
    if (b7 & 0x80) m |= btn::Triangle;
    if (b8 & 0x01) m |= btn::L1;
    if (b8 & 0x02) m |= btn::R1;
    if (b8 & 0x04) m |= btn::L2;
    if (b8 & 0x08) m |= btn::R2;
    if (b8 & 0x10) m |= btn::Create;
    if (b8 & 0x20) m |= btn::Options;
    if (b8 & 0x40) m |= btn::L3;
    if (b8 & 0x80) m |= btn::R3;
    if (b9 & 0x01) m |= btn::Home;
    if (b9 & 0x02) m |= btn::Touchpad;
    if (b9 & 0x04) m |= btn::Mute;
    if (b9 & 0x10) m |= btn::EdgeLeftFn;
    if (b9 & 0x20) m |= btn::EdgeRightFn;
    if (b9 & 0x40) m |= btn::EdgeLeftPaddle;
    if (b9 & 0x80) m |= btn::EdgeRightPaddle;
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
    return ParseStatus::Ok;
}

inline bool parseInputReport(const uint8_t* buffer, size_t size, InputState& out) noexcept {
    return parseInputReportStatus(buffer, size, out) == ParseStatus::Ok;
}

// Small conveniences
constexpr bool isDown(const InputState& s, uint32_t buttonBits) noexcept {
    return (s.buttons & buttonBits) == buttonBits;
}
constexpr float stickToFloat(uint8_t v) noexcept { return (static_cast<float>(v) - 127.5f) / 127.5f; }  // -1..1
constexpr float triggerToFloat(uint8_t v) noexcept { return static_cast<float>(v) / 255.0f; }          // 0..1

// ---------------------------------------------------------------------------
// TripleBuffer: wait-free single-producer / single-consumer "latest value" mailbox.
// The producer always writes to a private slot and swaps it into the middle; the consumer
// swaps the middle into its own slot. Neither side ever waits or sees a torn value.
// ---------------------------------------------------------------------------
template <typename T>
class TripleBuffer {
    static_assert(std::is_trivially_copyable<T>::value, "T must be trivially copyable");

public:
    // Producer thread only.
    void publish(const T& value) noexcept {
        slots_[back_] = value;
        const uint32_t prev = middle_.exchange(back_ | kFresh, std::memory_order_acq_rel);
        back_ = prev & kIndexMask;
    }

    // Consumer thread only. Returns true and fills `out` if a value newer than the last
    // consumed one exists; otherwise returns false and leaves `out` untouched.
    bool consume(T& out) noexcept {
        if ((middle_.load(std::memory_order_acquire) & kFresh) == 0) return false;
        const uint32_t prev = middle_.exchange(front_, std::memory_order_acq_rel);
        front_ = prev & kIndexMask;
        out = slots_[front_];
        return true;
    }

private:
    static constexpr uint32_t kFresh = 4u;
    static constexpr uint32_t kIndexMask = 3u;

    T slots_[3]{};
    uint32_t back_ = 0;   // producer-owned slot
    uint32_t front_ = 2;  // consumer-owned slot
    alignas(64) std::atomic<uint32_t> middle_{1};  // slot index (+ kFresh bit when newly published)
};

// ---------------------------------------------------------------------------
// TelemetryChannel: what the HID reader thread publishes into.
//   publish()  never blocks: updates the latest-value mailbox and appends to the history ring.
//   latest()   newest snapshot since the last call (UI polling at its own rate).
//   drain()    every sample, oldest first (recording / filtering); drops are counted, not blocked on.
// Pass ringCapacity = 0 to disable the history ring entirely.
// ---------------------------------------------------------------------------
class TelemetryChannel {
public:
    explicit TelemetryChannel(size_t ringCapacity = 1024)
        : ringEnabled_(ringCapacity != 0), ring_(ringCapacity != 0 ? ringCapacity : 2) {}

    // Producer thread only.
    void publish(const InputState& s) noexcept {
        latest_.publish(s);
        if (ringEnabled_ && ring_.write(&s, 1) == 0) dropped_.fetch_add(1, std::memory_order_relaxed);
        published_.fetch_add(1, std::memory_order_relaxed);
    }

    // Consumer thread only.
    bool latest(InputState& out) noexcept { return latest_.consume(out); }
    size_t drain(InputState* dst, size_t maxCount) noexcept {
        return ringEnabled_ ? ring_.read(dst, maxCount) : 0;
    }

    // Any thread.
    uint64_t published() const noexcept { return published_.load(std::memory_order_relaxed); }
    uint64_t dropped() const noexcept { return dropped_.load(std::memory_order_relaxed); }

private:
    const bool ringEnabled_;
    TripleBuffer<InputState> latest_;
    SpscRingBuffer<InputState> ring_;
    std::atomic<uint64_t> published_{0};
    std::atomic<uint64_t> dropped_{0};
};

}  // namespace haptic
