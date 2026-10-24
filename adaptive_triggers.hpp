// adaptive_triggers.hpp : header-only, C++17, no dependencies.
//
// Serializes adaptive-trigger profiles into the 63-byte USB output report (ID 0x02)
// for a DualSense-style controller. Layout and effect encodings follow the
// community reverse-engineered documentation (Nielk1's TriggerEffectGenerator and
// the Game Controller Collective wiki "SetStateData" structure).
//
// Output report 0x02 (offsets include the report ID at byte 0):
//   [ 0]      report ID 0x02
//   [ 1]      valid_flag0  bit2 (0x04) = apply right trigger effect
//                          bit3 (0x08) = apply left  trigger effect
//   [ 2]      valid_flag1  (untouched here: no lightbar/audio/LED changes requested)
//   [ 3..10]  rumble emulation / audio / mute fields (left zero, flags clear)
//   [11..21]  right trigger effect block (11 bytes: mode + 10 parameter bytes)
//   [22..32]  left  trigger effect block (11 bytes)
//   [33..62]  host timestamp, motor power, valid_flag2, LED fields, padding (zero)
//
// Effect blocks use "zones": the trigger travel is divided into 10 zones (0..9).
// USB reports carry no CRC. Bluetooth (report 0x31) needs a CRC32 and is out of scope.
//
// Threading: everything here is pure and allocation-free; call from any thread.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <type_traits>
#include <variant>

namespace haptic {

// ---------------------------------------------------------------------------
// Wire constants
// ---------------------------------------------------------------------------
constexpr size_t kTriggerBlockSize = 11;
constexpr size_t kOutputReportSize = 63;
constexpr uint8_t kOutputReportId = 0x02;
constexpr int kTriggerZones = 10;

namespace ds_report {
constexpr size_t kValidFlag0 = 1;
constexpr size_t kRightTrigger = 11;
constexpr size_t kLeftTrigger = 22;
constexpr uint8_t kFlag0AllowRightTrigger = 0x04;  // bit 2
constexpr uint8_t kFlag0AllowLeftTrigger = 0x08;   // bit 3
}  // namespace ds_report

// Mode byte (first byte of every trigger block). Only the three "official"
// effects plus Off are used here; they are the ones expected to survive firmware updates.
enum class TriggerMode : uint8_t {
    Off = 0x05,
    Feedback = 0x21,   // resistance, 10 per-zone strengths
    Weapon = 0x25,     // resistance between two zones, then release
    Vibration = 0x26,  // vibration beyond a start zone
};

using TriggerBlock = std::array<uint8_t, kTriggerBlockSize>;
using OutputReport = std::array<uint8_t, kOutputReportSize>;

// ---------------------------------------------------------------------------
// Profiles (what the application UI pushes)
// ---------------------------------------------------------------------------
struct OffProfile {};

// 1. Linear resistance: force ramps from startStrength at startZone to endStrength
//    at endZone, then holds endStrength to the end of travel.
struct LinearResistanceProfile {
    uint8_t startZone = 2;      // 0..8
    uint8_t endZone = 8;        // startZone+1..9
    uint8_t startStrength = 1;  // 1..8
    uint8_t endStrength = 8;    // 1..8
};

// 2. Clicky switch: resistance from startZone up to snapZone, then the force drops
//    to zero immediately, like a mechanical button breaking over.
struct ClickySwitchProfile {
    uint8_t startZone = 3;  // 2..7  (where resistance begins)
    uint8_t snapZone = 6;   // startZone+1..8 (break point; force releases past here)
    uint8_t strength = 8;   // 0..8 (0 = effect off)
};

// 3. Vibration: trigger vibrates at frequencyHz / amplitude once pulled past startZone.
struct VibrationProfile {
    uint8_t startZone = 4;      // 0..9 (threshold)
    uint8_t amplitude = 4;      // 0..8 (0 = effect off)
    uint8_t frequencyHz = 40;   // 0..255 (0 = effect off)
};

using TriggerProfile =
    std::variant<OffProfile, LinearResistanceProfile, ClickySwitchProfile, VibrationProfile>;

// nullopt  = leave this trigger alone (its valid_flag0 bit stays clear).
// OffProfile = explicitly switch the effect off (bit set, mode Off).
struct OutputState {
    std::optional<TriggerProfile> left;
    std::optional<TriggerProfile> right;
};

enum class SerializeStatus {
    Ok,
    InvalidLeftTrigger,
    InvalidRightTrigger,
};

// ---------------------------------------------------------------------------
// Encoding internals
// ---------------------------------------------------------------------------
namespace detail {

inline TriggerBlock offBlock() noexcept {
    TriggerBlock b{};
    b[0] = static_cast<uint8_t>(TriggerMode::Off);
    return b;
}

// Packs per-zone strengths (0 = inactive, 1..8 active) into the shared layout:
//   [1..2] = 10-bit active-zone mask, little endian
//   [3..6] = 3 bits per zone of (strength - 1), little endian
//   [9]    = frequency (Vibration only)
inline TriggerBlock packZones(TriggerMode mode, const std::array<uint8_t, kTriggerZones>& strength,
                              uint8_t frequency = 0) noexcept {
    uint16_t active = 0;
    uint32_t force = 0;
    for (int i = 0; i < kTriggerZones; ++i) {
        if (strength[i] > 0) {
            active = static_cast<uint16_t>(active | (1u << i));
            force |= static_cast<uint32_t>((strength[i] - 1) & 0x07) << (3 * i);
        }
    }
    TriggerBlock b{};
    b[0] = static_cast<uint8_t>(mode);
    b[1] = static_cast<uint8_t>(active & 0xFF);
    b[2] = static_cast<uint8_t>(active >> 8);
    b[3] = static_cast<uint8_t>(force & 0xFF);
    b[4] = static_cast<uint8_t>((force >> 8) & 0xFF);
    b[5] = static_cast<uint8_t>((force >> 16) & 0xFF);
    b[6] = static_cast<uint8_t>((force >> 24) & 0xFF);
    b[9] = frequency;
    return b;
}

inline bool encode(const OffProfile&, TriggerBlock& out) noexcept {
    out = offBlock();
    return true;
}

inline bool encode(const LinearResistanceProfile& p, TriggerBlock& out) noexcept {
    if (p.startZone > 8 || p.endZone > 9 || p.endZone <= p.startZone) return false;
    if (p.startStrength < 1 || p.startStrength > 8) return false;
    if (p.endStrength < 1 || p.endStrength > 8) return false;

    std::array<uint8_t, kTriggerZones> s{};
    const double slope = double(int(p.endStrength) - int(p.startStrength)) /
                         double(int(p.endZone) - int(p.startZone));
    for (int i = p.startZone; i < kTriggerZones; ++i) {
        s[i] = (i <= p.endZone)
                   ? static_cast<uint8_t>(std::nearbyint(p.startStrength + slope * (i - p.startZone)))  // half-to-even, matches reference
                   : p.endStrength;
    }
    out = packZones(TriggerMode::Feedback, s);
    return true;
}

inline bool encode(const ClickySwitchProfile& p, TriggerBlock& out) noexcept {
    if (p.startZone < 2 || p.startZone > 7) return false;
    if (p.snapZone > 8 || p.snapZone <= p.startZone) return false;
    if (p.strength > 8) return false;
    if (p.strength == 0) {
        out = offBlock();
        return true;
    }
    const uint16_t zones = static_cast<uint16_t>((1u << p.startZone) | (1u << p.snapZone));
    out = TriggerBlock{};
    out[0] = static_cast<uint8_t>(TriggerMode::Weapon);
    out[1] = static_cast<uint8_t>(zones & 0xFF);
    out[2] = static_cast<uint8_t>(zones >> 8);
    out[3] = static_cast<uint8_t>(p.strength - 1);
    return true;
}

inline bool encode(const VibrationProfile& p, TriggerBlock& out) noexcept {
    if (p.startZone > 9 || p.amplitude > 8) return false;
    if (p.amplitude == 0 || p.frequencyHz == 0) {
        out = offBlock();
        return true;
    }
    std::array<uint8_t, kTriggerZones> s{};
    for (int i = p.startZone; i < kTriggerZones; ++i) s[i] = p.amplitude;
    out = packZones(TriggerMode::Vibration, s, p.frequencyHz);
    return true;
}

}  // namespace detail

// Encodes one profile into an 11-byte trigger block. Returns false (and leaves
// `out` untouched) if any parameter is outside its documented range.
inline bool encodeTriggerProfile(const TriggerProfile& profile, TriggerBlock& out) noexcept {
    return std::visit([&out](const auto& p) { return detail::encode(p, out); }, profile);
}

// Builds the 63-byte USB output report 0x02. On error `out` is not modified, so a
// bad UI value can never produce a half-written report.
inline SerializeStatus serializeOutputReport(const OutputState& state, OutputReport& out) noexcept {
    TriggerBlock left{}, right{};
    if (state.left && !encodeTriggerProfile(*state.left, left))
        return SerializeStatus::InvalidLeftTrigger;
    if (state.right && !encodeTriggerProfile(*state.right, right))
        return SerializeStatus::InvalidRightTrigger;

    out.fill(0);
    out[0] = kOutputReportId;

    uint8_t flag0 = 0;
    if (state.right) {
        flag0 |= ds_report::kFlag0AllowRightTrigger;
        std::copy(right.begin(), right.end(), out.begin() + ds_report::kRightTrigger);
    }
    if (state.left) {
        flag0 |= ds_report::kFlag0AllowLeftTrigger;
        std::copy(left.begin(), left.end(), out.begin() + ds_report::kLeftTrigger);
    }
    out[ds_report::kValidFlag0] = flag0;
    return SerializeStatus::Ok;
}

}  // namespace haptic
