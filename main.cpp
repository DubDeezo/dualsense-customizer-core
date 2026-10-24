// main.cpp : DualSense Graphical GUI Dashboard Panel
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <vector>

#include "imgui.h"
#include "dualsense_manager.hpp"

using namespace haptic;
using namespace std::chrono_literals;

void RenderDualSenseDashboard(DualSenseManager& manager, InputState& telemetry, DualSenseManager::Stats& stats) {
    static int leftModeSelection = 0;  
    static int rightModeSelection = 0;

    static LinearResistanceProfile leftLinear, rightLinear;
    static ClickySwitchProfile leftClicky, rightClicky;
    static VibrationProfile leftVibe, rightVibe;
    static float hapticMasterDrive = 2.0f;

    static std::vector<float> gyroXHistory(128, 0.0f);
    static std::vector<float> gyroYHistory(128, 0.0f);
    static std::vector<float> r2History(128, 0.0f);
    static size_t historyOffset = 0;

    if (manager.connected()) {
        // FIX: Explicitly target array indices [0] and [1] to map specific sensor axes
        gyroXHistory[historyOffset] = static_cast<float>(telemetry.gyro[0]);
        gyroYHistory[historyOffset] = static_cast<float>(telemetry.gyro[1]);
        r2History[historyOffset]    = triggerToFloat(telemetry.r2);
        historyOffset = (historyOffset + 1) % 128;
    }

    ImGui::Begin("DualSense Advanced Customizer Dashboard Panel");

    ImGui::Text("Hardware Link: %s", manager.connected() ? "CONNECTED (USB)" : "DISCONNECTED / POLLING...");
    ImGui::SameLine(ImGui::GetWindowWidth() - 180.0f);
    ImGui::Text("Battery Status: %d%%", (int)telemetry.batteryLevel * 10);
    ImGui::Separator();

    ImGui::Columns(2, "MainLayoutColumns", true);

    ImGui::TextColored(ImVec4(0.3f, 0.8f, 1.0f, 1.0f), "Adaptive Trigger Configuration Profiles");
    ImGui::Spacing();

    ImGui::PushID("LeftTriggerZone");
    ImGui::Text("Left Trigger Profile Selector:");
    const char* modes[] = { "Off / Neutral", "Linear Resistance", "Tactile Clicky Switch", "Cyclic Vibration" };
    if (ImGui::Combo("Mode##Left", &leftModeSelection, modes, 4)) {
        if (leftModeSelection == 0) manager.setLeftTriggerProfile(OffProfile{});
    }

    uint8_t minZone = 0, maxZone = 9, minForce = 1, maxForce = 8;

    if (leftModeSelection == 1) { 
        bool updated = false;
        updated |= ImGui::SliderScalar("Start Zone##L", ImGuiDataType_U8, &leftLinear.startZone, &minZone, &maxZone);
        updated |= ImGui::SliderScalar("End Zone##L", ImGuiDataType_U8, &leftLinear.endZone, &minZone, &maxZone);
        updated |= ImGui::SliderScalar("Start Force##L", ImGuiDataType_U8, &leftLinear.startStrength, &minForce, &maxForce);
        updated |= ImGui::SliderScalar("End Force##L", ImGuiDataType_U8, &leftLinear.endStrength, &minForce, &maxForce);
        if (updated) manager.setLeftTriggerProfile(leftLinear);
    }
    else if (leftModeSelection == 2) { 
        bool updated = false;
        updated |= ImGui::SliderScalar("Snap Point##L", ImGuiDataType_U8, &leftClicky.startZone, &minZone, &maxZone);
        updated |= ImGui::SliderScalar("Break Depth##L", ImGuiDataType_U8, &leftClicky.snapZone, &minZone, &maxZone);
        updated |= ImGui::SliderScalar("Resistance##L", ImGuiDataType_U8, &leftClicky.strength, &minZone, &maxForce);
        if (updated) manager.setLeftTriggerProfile(leftClicky);
    }
    else if (leftModeSelection == 3) { 
        bool updated = false;
        updated |= ImGui::SliderScalar("Threshold##L", ImGuiDataType_U8, &leftVibe.startZone, &minZone, &maxZone);
        updated |= ImGui::SliderScalar("Amplitude##L", ImGuiDataType_U8, &leftVibe.amplitude, &minZone, &maxForce);
        uint8_t maxHz = 255;
        updated |= ImGui::SliderScalar("Frequency (Hz)##L", ImGuiDataType_U8, &leftVibe.frequencyHz, &minZone, &maxHz);
        if (updated) manager.setLeftTriggerProfile(leftVibe);
    }
    ImGui::PopID();
    ImGui::Separator();

    ImGui::PushID("RightTriggerZone");
    ImGui::Text("Right Trigger Profile Selector:");
    if (ImGui::Combo("Mode##Right", &rightModeSelection, modes, 4)) {
        if (rightModeSelection == 0) manager.setRightTriggerProfile(OffProfile{});
    }

    if (rightModeSelection == 1) {
        bool updated = false;
        updated |= ImGui::SliderScalar("Start Zone##R", ImGuiDataType_U8, &rightLinear.startZone, &minZone, &maxZone);
        updated |= ImGui::SliderScalar("End Zone##R", ImGuiDataType_U8, &rightLinear.endZone, &minZone, &maxZone);
        updated |= ImGui::SliderScalar("Start Force##R", ImGuiDataType_U8, &rightLinear.startStrength, &minForce, &maxForce);
        updated |= ImGui::SliderScalar("End Force##R", ImGuiDataType_U8, &rightLinear.endStrength, &minForce, &maxForce);
        if (updated) manager.setRightTriggerProfile(rightLinear);
    }
    else if (rightModeSelection == 2) {
        bool updated = false;
        updated |= ImGui::SliderScalar("Snap Point##R", ImGuiDataType_U8, &rightClicky.startZone, &minZone, &maxZone);
        updated |= ImGui::SliderScalar("Break Depth##R", ImGuiDataType_U8, &rightClicky.snapZone, &minZone, &maxZone);
        updated |= ImGui::SliderScalar("Resistance##R", ImGuiDataType_U8, &rightClicky.strength, &minZone, &maxForce);
        if (updated) manager.setRightTriggerProfile(rightClicky);
    }
    else if (rightModeSelection == 3) {
        bool updated = false;
        updated |= ImGui::SliderScalar("Threshold##R", ImGuiDataType_U8, &rightVibe.startZone, &minZone, &maxZone);
        updated |= ImGui::SliderScalar("Amplitude##R", ImGuiDataType_U8, &rightVibe.amplitude, &minZone, &maxForce);
        uint8_t maxHz = 255;
        updated |= ImGui::SliderScalar("Frequency (Hz)##R", ImGuiDataType_U8, &rightVibe.frequencyHz, &minZone, &maxHz);
        if (updated) manager.setRightTriggerProfile(rightVibe);
    }
    ImGui::PopID();
    ImGui::Separator();

    ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f), "Voice-Coil Actuator Stream Adjustments");
    if (ImGui::SliderFloat("Master Haptic Drive Gain", &hapticMasterDrive, 0.0f, 4.0f, "%.1fx")) {
        manager.haptics().setDrive(hapticMasterDrive);
    }

    ImGui::NextColumn();

    ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "Real-Time Telemetry Data Graphs");
    ImGui::Spacing();

    float gyroXData[128], gyroYData[128], r2Data[128]; // Fixed stack array data allocation layout bounds
    for (int i = 0; i < 128; ++i) {
        size_t idx = (historyOffset + i) % 128;
        gyroXData[i] = gyroXHistory[idx];
        gyroYData[i] = gyroYHistory[idx];
        r2Data[i]    = r2History[idx];
    }

    ImGui::Text("Gyroscopic Axis Sensor Fluctuations (X/Y Data Stream):");
    ImGui::PlotLines("Gyro X Axis##Plot", gyroXData, 128, 0, nullptr, -2000.0f, 2000.0f, ImVec2(0, 75));
    ImGui::PlotLines("Gyro Y Axis##Plot", gyroYData, 128, 0, nullptr, -2000.0f, 2000.0f, ImVec2(0, 75));
    ImGui::Spacing();

    ImGui::Text("Right Trigger Depth Tracking (Analog Ingestion Loop):");
    ImGui::PlotHistogram("R2 Pressure Depth##Histogram", r2Data, 128, 0, nullptr, 0.0f, 1.0f, ImVec2(0, 75));
    ImGui::Separator();

    ImGui::Text("I/O Core Diagnostic Analytics Summary Metrics:");
    ImGui::Columns(2, "StatsSubColumns", false);
    ImGui::Text("HID Packets Written: %llu", (unsigned long long)stats.reportsWritten);
    ImGui::Text("HID Inbound Reports: %llu", (unsigned long long)stats.inputReports);
    ImGui::NextColumn();
    ImGui::Text("Driver Link Drops: %llu", (unsigned long long)stats.connects - 1);
    ImGui::Text("Hardware Core Drops: %llu", (unsigned long long)stats.inputDropped);

    ImGui::Columns(1);
    ImGui::End();
}

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
    std::cout << "[SYSTEM] Close this terminal or press Ctrl+C to drop application threads.\n";

    while (true) {
        if (manager.connected()) {
            manager.getLatestInput(telemetry);
        }
        driverStats = manager.stats();

        RenderDualSenseDashboard(manager, telemetry, driverStats);

        std::this_thread::sleep_for(16.67ms); 
    }

    manager.stop();
    return 0;
}
