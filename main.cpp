#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <string>

#include "imgui.h"
#include "dualsense_manager.hpp"

using namespace haptic;
using namespace std::chrono_literals;

DualSenseManager* g_manager = nullptr;
InputState g_telemetry;
DualSenseManager::Stats g_stats;

LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
    switch (uMsg) {
        case WM_PAINT: {
            PAINTSTRUCT ps;
            HDC hdc = BeginPaint(hwnd, &ps);
            
            HBRUSH bgBrush = CreateSolidBrush(RGB(24, 24, 24));
            FillRect(hdc, &ps.rcPaint, bgBrush);
            DeleteObject(bgBrush);

            HBRUSH panelBrush = CreateSolidBrush(RGB(45, 45, 45));
            RECT leftPanel = { 20, 70, 480, 570 };
            RECT rightPanel = { 520, 70, 980, 570 };
            FillRect(hdc, &leftPanel, panelBrush);
            FillRect(hdc, &rightPanel, panelBrush);
            DeleteObject(panelBrush);

            SetBkMode(hdc, TRANSPARENT);
            
            HFONT hFontTitle = CreateFontW(24, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE, ANSI_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, DEFAULT_QUALITY, DEFAULT_PITCH | FF_SWISS, L"Segoe UI");
            HFONT hOldFont = (HFONT)SelectObject(hdc, hFontTitle);
            
            SetTextColor(hdc, RGB(255, 255, 255));
            std::wstring title = L"DualSense Advanced Hardware Control Panel";
            TextOutW(hdc, 30, 20, title.c_str(), static_cast<int>(title.length()));

            HFONT hFontNormal = CreateFontW(16, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, ANSI_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, DEFAULT_QUALITY, DEFAULT_PITCH | FF_SWISS, L"Segoe UI");
            SelectObject(hdc, hFontNormal);

            SetTextColor(hdc, RGB(64, 192, 255));
            std::wstring lTitle = L"Adaptive Trigger Profiles Configuration";
            TextOutW(hdc, 40, 90, lTitle.c_str(), static_cast<int>(lTitle.length()));

            SetTextColor(hdc, RGB(240, 240, 240));
            std::wstring i1 = L"- Hold [L1]: sniper steady pressure linear curve profile";
            std::wstring i2 = L"- Press [Cross]: automatic rapid cyclic weapon vibration";
            std::wstring i3 = L"- Neutral: default tactile mechanical clicky switch break";
            TextOutW(hdc, 40, 140, i1.c_str(), static_cast<int>(i1.length()));
            TextOutW(hdc, 40, 170, i2.c_str(), static_cast<int>(i2.length()));
            TextOutW(hdc, 40, 200, i3.c_str(), static_cast<int>(i3.length()));

            SetTextColor(hdc, RGB(102, 255, 102));
            std::wstring rTitle = L"Real-Time Telemetry Performance Analytics";
            TextOutW(hdc, 540, 90, rTitle.c_str(), static_cast<int>(rTitle.length()));

            SetTextColor(hdc, RGB(255, 255, 255));
            
            std::wstring s1 = L"Outbound HID Written Packets: " + std::to_wstring(g_stats.reportsWritten);
            std::wstring s2 = L"Inbound HID Received Frames: " + std::to_wstring(g_stats.inputReports);
            std::wstring s3 = L"Battery Capacity Indicator: " + std::to_wstring(g_telemetry.batteryLevel * 10) + L"% Status";
            std::wstring s4 = L"Simulated Left Stick Position: X=" + std::to_wstring(g_telemetry.leftStickX) + L" Y=" + std::to_wstring(g_telemetry.leftStickY);

            TextOutW(hdc, 540, 140, s1.c_str(), static_cast<int>(s1.length()));
            TextOutW(hdc, 540, 170, s2.c_str(), static_cast<int>(s2.length()));
            TextOutW(hdc, 540, 200, s3.c_str(), static_cast<int>(s3.length()));
            TextOutW(hdc, 540, 240, s4.c_str(), static_cast<int>(s4.length()));

            SelectObject(hdc, hOldFont);
            DeleteObject(hFontTitle);
            DeleteObject(hFontNormal);

            EndPaint(hwnd, &ps);
            return 0;
        }
        case WM_CLOSE:
            DestroyWindow(hwnd);
            return 0;
        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;
    }
    return DefWindowProc(hwnd, uMsg, wParam, lParam);
}

int main() {
    DualSenseConfig config;
    config.maxUpdateHz = 200.0;
    config.readTimeoutMs = 5;

    DualSenseManager manager(config);
    g_manager = &manager;

    if (!manager.start()) {
        std::cerr << "[FATAL] Failed to link low level controller thread channels.\n";
        return 1;
    }

    HINSTANCE hInstance = GetModuleHandle(nullptr);
    WNDCLASSW wc = {};
    wc.lpfnWndProc   = WindowProc;
    wc.hInstance     = hInstance;
    wc.lpszClassName = L"DualSenseDashboardClass";
    wc.hCursor       = LoadCursor(nullptr, IDC_ARROW);

    RegisterClassW(&wc);

    HWND hwnd = CreateWindowExW(
        0, L"DualSenseDashboardClass", L"DualSense Hardware Manager Panel",
        WS_OVERLAPPEDWINDOW & ~WS_THICKFRAME & ~WS_MAXIMIZEBOX, 
        CW_USEDEFAULT, CW_USEDEFAULT, 1020, 640,
        nullptr, nullptr, hInstance, nullptr
    );

    if (!hwnd) {
        std::cerr << "[FATAL] Window container initialization failed.\n";
        manager.stop();
        return 1;
    }

    ShowWindow(hwnd, SW_SHOWNORMAL);
    UpdateWindow(hwnd);

    MSG msg = {};
    bool running = true;

    manager.releaseTriggers();

    while (running) {
        while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) {
                running = false;
            }
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }

        if (!running) break;

        if (manager.connected()) {
            manager.getLatestInput(g_telemetry);
        }
        g_stats = manager.stats();

        InvalidateRect(hwnd, nullptr, TRUE);
        std::this_thread::sleep_for(16ms);
    }

    manager.stop();
    return 0;
}
