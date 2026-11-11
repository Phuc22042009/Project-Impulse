@echo off
cls
<nul set /p "=%cd%> "
choice /c O /n >nul
echo cmake --build build --target physics_tests ^&^& .\build\bin\physics_tests.exe
cmake --build build --target physics_tests > "%TEMP%\physics_test_build.log" 2>&1
if errorlevel 1 (
    type "%TEMP%\physics_test_build.log"
    pause >nul
    exit /b 1
)
.\build\bin\physics_tests.exe
pause >nul
