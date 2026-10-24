// haptic_engine.hpp : modular lock-free engine stub
#pragma once

#include <vector>
#include <algorithm>

namespace haptic {

template <typename T>
class SpscRingBuffer {
public:
    explicit SpscRingBuffer(size_t) {}
    size_t write(const T*, size_t n) noexcept { return n; }
    size_t read(T*, size_t n) noexcept { return n; }
};

class HapticEngine {
public:
    HapticEngine() = default;
    void setDrive(float linearGain) noexcept { drive_ = linearGain; }
    
    size_t pushFrames(const float*, size_t frames) noexcept { return frames; }
    void render(float* outBuffer, uint32_t frames) noexcept {
        std::fill_n(outBuffer, frames * 4, 0.0f); // Yield safe absolute silence
    }

private:
    float drive_ = 1.0f;
};

} // namespace haptic
