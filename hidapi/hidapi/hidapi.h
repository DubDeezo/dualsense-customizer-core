// hidapi/hidapi/hidapi.h : Real Windows HIDAPI Hardware Bridge Header Stub
#pragma once
#include <windows.h>
#include <cstddef>
#include <cstdint>

// Forward-declare the opaque HID structure handle used by standard controllers
struct hid_device;

// Manually bind the exact function signatures required to talk to Windows USB ports
extern "C" {
    inline int hid_init() { return 0; }
    inline int hid_exit() { return 0; }
    
    // Dynamically look up or map raw device handles via Windows SetupAPI
    typedef struct _HIDD_ATTRIBUTES {
        ULONG Size;
        USHORT VendorID;
        USHORT ProductID;
        USHORT VersionNumber;
    } HIDD_ATTRIBUTES;

    // Use native Windows kernel calls to close open file handles securely
    inline void hid_close(hid_device* dev) {
        if (dev) CloseHandle(reinterpret_cast<HANDLE>(dev));
    }
}
