// hidapi.h : Universal Windows HIDAPI controller emulation header stub
#pragma once
#include <cstddef>
#include <cstdint>

struct hid_device;

inline int hid_init() { return 0; }
inline int hid_exit() { return 0; }
inline void hid_close(hid_device*) {}

inline hid_device* hid_open(unsigned short, unsigned short, const wchar_t*) {
    // Uses a safe 64-bit integer cast to satisfy modern x64 architecture compilation rules
    return reinterpret_cast<hid_device*>(static_cast<uintptr_t>(0xDEADBEEF)); 
}

inline int hid_write(hid_device*, const unsigned char*, size_t) { return 63; }
inline int hid_read_timeout(hid_device*, unsigned char* data, size_t, int) {
    if (!data) return -1;
    data[0] = 0x01; // Input Report ID
    data[1] = 128;  // Left stick X
    data[2] = 128;  // Left stick Y
    data[3] = 128;  // Right stick X
    data[4] = 128;  // Right stick Y
    return 64;      
}
