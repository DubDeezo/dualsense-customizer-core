@echo off
echo =============================================================
echo 🎮 DUALSENSE CORE: HARDWARE UNLOCK AND COMPILATION LOOP
echo =============================================================

taskkill /f /im dualsense_dashboard.exe >nul 2>&1

if exist build (
    attrib -r -s -h build /s /d >nul 2>&1
    rmdir /s /q build >nul 2>&1
)

if exist build (
    rename build build_garbage >nul 2>&1
    rmdir /s /q build_garbage >nul 2>&1
)

mkdir build
cd build

cmake -DCMAKE_BUILD_TYPE=Release ..
if %errorlevel% neq 0 (
    echo [ERROR] Build layout architecture configuration failed!
    pause
    exit /b %errorlevel%
)

cmake --build . --config Release
if %errorlevel% neq 0 (
    echo [ERROR] Compilation pass broken!
    pause
    exit /b %errorlevel%
)

echo [SUCCESS] Build target completed perfectly!
echo -------------------------------------------------------------
cd Release
dualsense_dashboard.exe
pause
