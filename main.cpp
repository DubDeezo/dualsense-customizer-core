Below is a complete, minimal simulation demonstrating how to initialize the core manager dashboard, capture wait-free controller button inputs, and push high-frequency tactical weapon behavior modifiers directly to the controller triggers based on active state conditions:

#include <iostream>
#include <chrono>
#include <thread>
#include "dualsense_manager.hpp"

using namespace haptic;
using namespace std::chrono_literals;

int main() {
    DualSenseConfig config;
    config.maxUpdateHz = 200.0; // Responsive rate limiting for trigger data
    config.readTimeoutMs = 5;   // Low latency shutdown tracking boundary
    
    DualSenseManager manager(config);
    
    std::cout << "[SYSTEM] Initializing DualSense Engine Subsystems...\n";
    if (!manager.start()) {
        std::cerr << "[FATAL] Failed to initialize hardware interface.\n";
        return 1;
    }
    
    std::cout << "[SYSTEM] Polling for DualSense hardware. Connect over USB now...\n";
    
    InputState state;
    bool lastL1State = false;
    bool lastCrossState = false;

    // Standard application main rendering loop simulation (60Hz execution context)
    while (true) {
        if (manager.connected()) {
            // Ingest the latest hardware parsing state snapshot (wait-free)
            if (manager.getLatestInput(state)) {
                
                bool currentL1 = isDown(state, btn::L1);
                bool currentCross = isDown(state, btn::Cross);
                
                // FEATURE 1: Tactical "Aim Down Sights" (ADS) Mode via L1
                if (currentL1 != lastL1State) {
                    lastL1State = currentL1;
                    if (currentL1) {
                        std::cout << "[INPUT] L1 Pressed: Engaging Sniper Steady-Pressure Resistance.\n";
                        LinearResistanceProfile adsProfile;
                        adsProfile.startZone = 1;
                        adsProfile.endZone = 5;
                        adsProfile.startStrength = 3;
                        adsProfile.endStrength = 8;
                        manager.setLeftTriggerProfile(adsProfile);
                    } else {
                        std::cout << "[INPUT] L1 Released: Restoring Left Trigger to Neutral.\n";
                        manager.setLeftTriggerProfile(OffProfile{});
                    }
                }
                
                // FEATURE 2: Adaptive Fire Weapon Selection via Cross Button
                if (currentCross && !lastCrossState) {
                    lastCrossState = true;
                    std::cout << "[INPUT] Cross Pressed: Equipping Automatic Cyclic Weapon Vibrations.\n";
                    VibrationProfile fullAuto;
                    fullAuto.startZone = 2;
                    fullAuto.amplitude = 7;
                    fullAuto.frequencyHz = 45; 
                    manager.setRightTriggerProfile(fullAuto);
                } 
                else if (!currentCross && lastCrossState) {
                    lastCrossState = false;
                    std::cout << "[INPUT] Cross Released: Swapping to Semi-Auto Tactile Clicky Trigger.\n";
                    ClickySwitchProfile semiAuto;
                    semiAuto.startZone = 3;
                    semiAuto.snapZone = 6;
                    semiAuto.strength = 8; // Mechanical breakpoint tactile snap
                    manager.setRightTriggerProfile(semiAuto);
                }
                
                // Live telemetry diagnostics tracking
                std::cout << "[TELEM] L_Stick: (" << stickToFloat(state.leftStickX) 
                          << ", " << stickToFloat(state.leftStickY) 
                          << ") | R2 Depth: " << (int)state.r2 
                          << " | Battery Status: " << (int)state.batteryLevel * 10 << "%\r" << std::flush;
            }
        } else {
            std::cout << "[ALERT] Connection broken. Waiting for hardware polling hook...\r" << std::flush;
            std::this_thread::sleep_for(500ms);
        }
        
        std::this_thread::sleep_for(16ms); // Target standard 60 FPS pacing update window
    }
    
    manager.stop();
    return 0;
}
