# 🎮 DualSense Advanced Customizer & Haptic Synthesizer Core

A performance-focused, lock-free, zero-allocation C++17 library designed to unlock advanced customization for the Sony DualSense controller. This project decouples high-frequency hardware interaction from application rendering loops, allowing you to manipulate adaptive trigger data payloads and stream digital signal processed (DSP) audio waveforms directly to the controller's voice-coil haptic actuators over a USB HID connection.

## 🚀 Key Features

*   **Audio-to-Haptic DSP Pipeline:** Streams raw interleaved stereo audio frames into a lock-free Single-Producer Single-Consumer (SPSC) ring buffer. Low-pass filters sub-sonic elements via a cascading 4th-order Butterworth filter (cutoff at 400Hz), blocks DC offset at 20Hz, and utilizes a stereo-linked soft-knee compressor with an instant-attack brickwall limiter to maximize vibration intensity while safely protecting the controller's physical internal voice coils.
*   **Bit-Packed Adaptive Trigger Serializer:** Bypasses proprietary API abstractions to build bare-metal 63-byte USB output reports (`0x02`). It handles 10-bit little-endian active zone bitmasking and custom parameter encoding for Linear Resistance curves, automatic cyclic weapon vibration frequencies, and crisp physical mechanical switch breakpoint drops.
*   **Wait-Free Telemetry Channel:** Decodes the 64-byte USB input report (`0x01`). Extracts 12-bit tightly packed DualSense touchpad coordinates, raw 3-axis gyro and accelerometer readings, analog trigger depth values, and discrete digital button states. It implements a thread-safe, wait-free **Triple Buffer** mailbox structure so that a high-speed hardware reading thread can ingest data at 250Hz–1000Hz without ever stalling a 60Hz UI visualization or game simulation layer.
*   **Asynchronous DualSense Coordinator:** Managed under RAII lifecycle boundaries. Spawns dedicated background worker threads to isolate thread operations, prevents HID packet flooding via a configurable micro-second rate limiter (clamped safely between 10Hz and 250Hz), and implements hot-plug auto-reconnect logic that re-syncs state states seamlessly if the physical hardware connection drops.

---

## 📦 Project Architecture & Layout

The project is structured as a lightweight, single-responsibility header-only core managed by a central manager:

| Component | Responsibility | Performance Contract |
| :--- | :--- | :--- |
| **`haptic_engine.hpp`** | Lock-free SPSC buffer, Cascaded Biquad Filters, Compressor, Hard Ceiling Limiter, Actuator Routing. | Zero heap allocations, zero locks. Audio thread-safe. |
| **`adaptive_triggers.hpp`** | Variant-based trigger profiles, Bit-packing zone masks, 11-byte output serialization. | Pure math functions, allocation-free. Call from any thread. |
| **`dualsense_telemetry.hpp`** | 12-bit touchpad bit-shifting, input parsing, Triple Buffer wait-free state mailbox. | Zero locks. Hardware consumer/producer isolated. |
| **`dualsense_manager.hpp/.cpp`** | Asynchronous Reader/Writer thread loops, `hidapi` connection lifecycle, rate throttling. | Lock-free on execution hot-paths. Mutexes isolated to setup/shutdown. |

---

## 🛠️ Prerequisites & Installation

To compile and link the library, your workstation must have:
*   A C++17 compliant compiler (`GCC 9+`, `Clang 10+`, or `MSVC 2019+`)
*   `CMake 3.16` or newer
*   **`hidapi` library installed on your system:**
    *   **Ubuntu/Debian:** `sudo apt install libhidapi-dev`
    *   **macOS (Homebrew):** `brew install hidapi`
    *   **Windows (vcpkg):** `vcpkg install hidapi`

---

## 🔨 Building the Library

Create the standard CMake compilation directories inside your project folder:
```bash
mkdir build && cd build
cmake ..
cmake --build .
```

---

## 🔒 License
This core library framework is open-source hardware utility software. Use it responsibly to build next-generation immersive controller applications!
