@ECHO OFF
rd /s /q "cmake-build-debug" > nul 2>&1
rd /s /q "bin" > nul 2>&1
mkdir cmake-build-debug
cd cmake-build-debug
C:\root\usr\bin\cmake .. -G "CodeBlocks - MinGW Makefiles"
C:\root\usr\bin\mingw32-make -j8
cd ..