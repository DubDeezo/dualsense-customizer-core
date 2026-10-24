cpp
// main.cpp : DualSense Graphical GUI Dashboard Panel
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <vector>

// Include Dear ImGui Core System Structures
#include "imgui.h"

#include "dualsense_manager.hpp"

using namespace haptic;
using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Simulating an Embedded Application Interface Window Wrapper
// ---------------------------------------------------------------------------
void RenderDualSenseDashboard(DualSenseManager& manager, InputState& telemetry, DualSenseManager::Stats& stats) {
    // Authoritative Configuration State Variables
    static int leftModeSelection = 0;  // 0: Off, 1: Linear, 2: Clicky, 3: Vibe
    static int rightModeSelection = 0;

    static LinearResistanceProfile leftLinear, rightLinear;
    static ClickySwitchProfile leftClicky, rightClicky;
    static VibrationProfile leftVibe, rightVibe;
    static float hapticMasterDrive = 2.0f;

    // Rolling Telemetry Frame History Allocation Channels (Wait-Free Ring Buffer equivalent)
    static std::vector<float> gyroXHistory(128, 0.0f);
    static std::vector<float> gyroYHistory(128, 0.0f);
    static std::vector<float> r2History(128, 0.0f);
    static size_t historyOffset = 0;

    // Ingest latest hardware telemetry snapshot frames into rolling history blocks
    if (manager.connected()) {
        gyroXHistory[historyOffset] = static_cast<float>(telemetry.gyro[0]);
        gyroYHistory[historyOffset] = static_cast<float>(telemetry.gyro[1]);
        r2History[historyOffset]    = triggerToFloat(telemetry.r2);
        historyOffset = (historyOffset + 1) % 128;
    }

    // Begin Rendering Layout Architecture Panels
    ImGui::Begin("DualSense Advanced Customizer Dashboard Panel");

    // Top Status Metric Readout Strip
    ImGui::Text("Hardware Link: %s", manager.connected() ? "CONNECTED (USB)" : "DISCONNECTED / POLLING...");
    ImGui::SameLine(ImGui::GetWindowWidth() - 180.0f);
    ImGui::Text("Battery Status: %d%%", (int)telemetry.batteryLevel * 10);
    ImGui::Separator();

    // Render Split-Window Column Views
    ImGui::Columns(2, "MainLayoutColumns", true);

    // ==========================================
    // COLUMN 1: INTERACTIVE HARDWARE CONTROLS
    // ==========================================
    ImGui::TextColored(ImVec4(0.3f, 0.8f, 1.0f, 1.0f), "Adaptive Trigger Configuration Profiles");
    ImGui::Spacing();

    // ---- LEFT TRIGGER SECTION ----
    ImGui::PushID("LeftTriggerZone");
    ImGui::Text("Left Trigger Profile Selector:");
    const char* modes[] = { "Off / Neutral", "Linear Resistance", "Tactile Clicky Switch", "Cyclic Vibration" };
    if (ImGui::Combo("Mode##Left", &leftModeSelection, modes, IM_ARRAYSIZE(modes))) {
        if (leftModeSelection == 0) manager.setLeftTriggerProfile(OffProfile{});
    }

    if (leftModeSelection == 1) { // Linear Configuration sliders
        bool updated = false;
        updated |= ImGui::SliderScalar("Start Zone##L", ImGuiDataType_U8, &leftLinear.startZone, &"0", &"8");
        updated |= ImGui::SliderScalar("End Zone##L", ImGuiDataType_U8, &leftLinear.endZone, &"2", &"9");
        updated |= ImGui::SliderScalar("Start Force##L", ImGuiDataType_U8, &leftLinear.startStrength, &"1", &"8");
        updated |= ImGui::SliderScalar("End Force##L", ImGuiDataType_U8, &leftLinear.endStrength, &"1", &"8");
        if (updated) manager.setLeftTriggerProfile(leftLinear);
    }
    else if (leftModeSelection == 2) { // Clicky Config Panels
        bool updated = false;
        updated |= ImGui::SliderScalar("Snap Point##L", ImGuiDataType_U8, &leftClicky.startZone, &"2", &"7");
        updated |= ImGui::SliderScalar("Break Depth##L", ImGuiDataType_U8, &leftClicky.snapZone, &"3", &"8");
        updated |= ImGui::SliderScalar("Resistance##L", ImGuiDataType_U8, &leftClicky.strength, &"0", &"8");
        if (updated) manager.setLeftTriggerProfile(leftClicky);
    }
    else if (leftModeSelection == 3) { // Vibration Config Sliders
        bool updated = false;
        updated |= ImGui::SliderScalar("Threshold##L", ImGuiDataType_U8, &leftVibe.startZone, &"0", &"9");
        updated |= ImGui::SliderScalar("Amplitude##L", ImGuiDataType_U8, &leftVibe.amplitude, &"0", &"8");
        updated |= ImGui::SliderScalar("Frequency (Hz)##L", ImGuiDataType_U8, &leftVibe.frequencyHz, &"0", &"255");
        if (updated) manager.setLeftTriggerProfile(leftVibe);
    }
    ImGui::PopID();
    ImGui::Separator();

    // ---- RIGHT TRIGGER SECTION ----
    ImGui::PushID("RightTriggerZone");
    ImGui::Text("Right Trigger Profile Selector:");
    if (ImGui::Combo("Mode##Right", &rightModeSelection, modes, IM_ARRAYSIZE(modes))) {
        if (rightModeSelection == 0) manager.setRightTriggerProfile(OffProfile{});
    }

    if (rightModeSelection == 1) {
        bool updated = false;
        updated |= ImGui::SliderScalar("Start Zone##R", ImGuiDataType_U8, &rightLinear.startZone, &"0", &"8");
        updated |= ImGui::SliderScalar("End Zone##R", ImGuiDataType_U8, &rightLinear.endZone, &"2", &"9");
        updated |= ImGui::SliderScalar("Start Force##R", ImGuiDataType_U8, &rightLinear.startStrength, &"1", &"8");
        updated |= ImGui::SliderScalar("End Force##R", ImGuiDataType_U8, &rightLinear.endStrength, &"1", &"8");
        if (updated) manager.setRightTriggerProfile(rightLinear);
    }
    else if (rightModeSelection == 2) {
        bool updated = false;
        updated |= ImGui::SliderScalar("Snap Point##R", ImGuiDataType_U8, &rightClicky.startZone, &"2", &"7");
        updated |= ImGui::SliderScalar("Break Depth##R", ImGuiDataType_U8, &rightClicky.snapZone, &"3", &"8");
        updated |= ImGui::SliderScalar("Resistance##R", ImGuiDataType_U8, &rightClicky.strength, &"0", &"8");
        if (updated) manager.setRightTriggerProfile(rightClicky);
    }
    else if (rightModeSelection == 3) {
        bool updated = false;
        updated |= ImGui::SliderScalar("Threshold##R", ImGuiDataType_U8, &rightVibe.startZone, &"0", &"9");
        updated |= ImGui::SliderScalar("Amplitude##R", ImGuiDataType_U8, &rightVibe.amplitude, &"0", &"8");
        updated |= ImGui::SliderScalar("Frequency (Hz)##R", ImGuiDataType_U8, &rightVibe.frequencyHz, &"0", &"255");
        if (updated) manager.setRightTriggerProfile(rightVibe);
    }
    ImGui::PopID();
    ImGui::Separator();

    // ---- MASTER AUDIO HAPTICS GAIN CONTROL ----
    ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f), "Voice-Coil Actuator Stream Adjustments");
    if (ImGui::SliderFloat("Master Haptic Drive Gain", &hapticMasterDrive, 0.0f, 4.0f, "%.1fx")) {
        manager.haptics().setDrive(hapticMasterDrive);
    }

    // Advance Layout Alignment to Column 2
    ImGui::NextColumn();

    // ==========================================
    // COLUMN 2: LIVE DIAGNOSTIC OSCILLOSCOPES
    // ==========================================
    ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "Real-Time Telemetry Data Graphs");
    ImGui::Spacing();

    // Extract ordered historical snapshots to populate standard graph arrays
    float gyroXData[128], gyroYData[128], r2Data[128];
    for (int i = 0; i < 128; ++i) {
        size_t idx = (historyOffset + i) % 128;
        gyroXData[i] = gyroXHistory[idx];
        gyroYData[i] = gyroYHistory[idx];
        r2Data[i]    = r2History[idx];
    }

    // Draw Rolling Waveforms to Visualize Joystick Drift or Gyroscopic Fluctuations
    ImGui::Text("Gyroscopic Axis Sensor Fluctuations (X/Y Data Stream):");
    ImGui::PlotLines("Gyro X Axis##Plot", gyroXData, 128, 0, nullptr, -2000.0f, 2000.0f, ImVec2(0, 75));
    ImGui::PlotLines("Gyro Y Axis##Plot", gyroYData, 128, 0, nullptr, -2000.0f, 2000.0f, ImVec2(0, 75));
    ImGui::Spacing();

    ImGui::Text("Right Trigger Depth Tracking (Analog Ingestion Loop):");
    ImGui::PlotHistogram("R2 Pressure Depth##Histogram", r2Data, 128, 0, nullptr, 0.0f, 1.0f, ImVec2(0, 75));
    ImGui::Separator();

    // Quantitative Core Engine Analytics Metrics
    ImGui::Text("I/O Core Diagnostic Analytics Summary Metrics:");
    ImGui::Columns(2, "StatsSubColumns", false);
    ImGui::Text("HID Packets Written: %llu", (unsigned long long)stats.reportsWritten);
    ImGui::Text("HID Inbound Reports: %llu", (unsigned long long)stats.inputReports);
    ImGui::NextColumn();
    ImGui::Text("Driver Link Drops: %llu", (unsigned long long)stats.connects - 1);
    ImGui::Text("Hardware Core Drops: %llu", (unsigned long long)stats.inputDropped);

    // Reset layout system context constraints
    ImGui::Columns(1);
    ImGui::End();
}

// Simulated Executable Main Function Wrapper path
int main() {
    DualSenseConfig config;
    config.maxUpdateHz = 200.0;
    config.readTimeoutMs = 5;

    DualSenseManager manager(config);
    if (!manager.start()) {
        std::cerr << "[FATAL] Failed to initialize background driver communication.\n";
        return 1;
    }

    InputState telemetry;
    DualSenseManager::Stats driverStats;

    std::cout << "[SUCCESS] GUI Sub-Engine Active. Initializing Immediate Mode Rendering Frames...\n";
    std::cout << "[SYSTEM] Press Ctrl + C inside this shell environment context to drop application threads.\n";

    // Simulate standard native UI window lifecycle ticking at a constant 60 FPS update pace window
    while (true) {
        if (manager.connected()) {
            manager.getLatestInput(telemetry);
        }
        driverStats = manager.stats();

        // Simulate an ImGui Rendering Frame Dispatch Execution Boundary Pass
        // In a real application wrapper environment, your graphics card system backend (DirectX/OpenGL)
        // automatically wraps these functions inside an active window container wrapper.
        RenderDualSenseDashboard(manager, telemetry, driverStats);

        std::this_thread::sleep_for(16.67ms); 
    }

    manager.stop();
    return 0;
}