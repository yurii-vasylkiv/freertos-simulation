@ECHO OFF
rd /s /q "cmake-build-debug" > nul 2>&1
rd /s /q "bin" > nul 2>&1
rd /s /q "generated" > nul 2>&1
del BUILD.bat