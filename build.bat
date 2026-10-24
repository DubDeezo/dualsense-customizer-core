@echo off
echo =============================================================
echo 🎮 DUALSENSE CORE: HARDWARE UNLOCK AND COMPILATION LOOP
echo =============================================================

:: Force kill any stuck background dashboard processes
taskkill /f /im dualsense_dashboard.exe >nul 2>&1

:: Force clear Windows file locks on the build directory
echo [LOCK] Releasing hidden cloud file hooks...
if exist build (
    attrib -r -s -h build /s /d >nul 2>&1
    rmdir /s /q build >nul 2>&1
)

:: Alternative fallback rename-delete strategy if OneDrive is being stubborn
if exist build (
    rename build build_garbage >nul 2>&1
    rmdir /s /q build_garbage >nul 2>&1
)

echo [DIR] Allocating active compile cache targets...
mkdir build
cd build

echo [CONFIG] Binding build systems to native compilers...
cmake -DCMAKE_BUILD_TYPE=Release ..
if %errorlevel% neq 0 (
    echo [ERROR] Build layout architecture configuration failed!
    pause
    exit /b %errorlevel%
)

echo [COMPILE] Building core modules with multi-threaded performance...
cmake --build . --config Release
if %errorlevel% neq 0 (
    echo [ERROR] Compilation pass broken! Review source architecture errors.
    pause
    exit /b %errorlevel%
)

echo [SUCCESS] Build target completed perfectly!
echo [RUN] Launching Graphical Dashboard Panel Application Wrapper...
echo -------------------------------------------------------------
cd Release
dualsense_dashboard.exe
pause
