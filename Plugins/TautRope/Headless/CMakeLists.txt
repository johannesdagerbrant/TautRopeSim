# Builds the rope simulation and the replay program with no Unreal present.
#
# This exists so an agent can change core code, rebuild only core, and re-run a
# recording in seconds. Nothing here may reference the engine: if a UE header
# ever reaches TautRopeCore, this build is what breaks first.
#
#   cmake -S . -B build -G Ninja
#   cmake --build build
#
cmake_minimum_required(VERSION 3.20)
project(TautRopeHeadless CXX)

# C++20 to match what UBT gives the editor build (/std:c++20). Compiling the
# same sources under two language standards is a correctness hazard well beyond
# floating point.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

set(TAUTROPE_CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Source/TautRopeCore")

# Globbed with CONFIGURE_DEPENDS so adding a core file does not mean editing this
# file; CMake re-globs on build.
file(GLOB TAUTROPE_CORE_SOURCES CONFIGURE_DEPENDS "${TAUTROPE_CORE_DIR}/Private/*.cpp")
file(GLOB TAUTROPE_CORE_HEADERS CONFIGURE_DEPENDS "${TAUTROPE_CORE_DIR}/Public/TautRopeCore/*.h")

if(NOT TAUTROPE_CORE_SOURCES)
	message(FATAL_ERROR "No core sources found under ${TAUTROPE_CORE_DIR}/Private")
endif()

add_library(tautrope_core STATIC ${TAUTROPE_CORE_SOURCES} ${TAUTROPE_CORE_HEADERS})
target_include_directories(tautrope_core PUBLIC "${TAUTROPE_CORE_DIR}/Public")

# Matched against the flags UBT actually passes the editor build, read out of
# Intermediate/Build/Win64/x64/UnrealEditor/Development/TautRopeCore/*.rsp
# rather than assumed:
#
#   /fp:precise    same here. Anything else diverges from recordings.
#   no /arch:      same here, so both keep the x64 SSE2 baseline.
#   /GR-           same here. Irrelevant to results, but no reason to differ.
#   /std:c++20     set above.
#   /Zc:__cplusplus  same here; see below.
#   /Zp8           the x64 default, so nothing to do.
#   /Ox /Ob2       not matched. Optimisation level does not change results under
#                  /fp:precise, and forcing it would mean overriding CMake's
#                  per-config flags for no benefit.
#
# TautRopeCore/Core.h enforces the parts that matter with #error guards, so a
# drift here fails at compile time in whichever build drifted.
if(MSVC)
	# /Zc:__cplusplus because MSVC otherwise reports __cplusplus as 199711L
	# regardless of /std:, which would defeat the language-standard guard in
	# Core.h. UBT passes it too.
	target_compile_options(tautrope_core PUBLIC /fp:precise /GR- /Zc:__cplusplus)
	target_compile_options(tautrope_core PRIVATE /W4 /permissive-)
else()
	target_compile_options(tautrope_core PUBLIC -ffp-contract=off -fno-rtti)
	target_compile_options(tautrope_core PRIVATE -Wall -Wextra)
endif()

add_executable(tautrope-replay "${CMAKE_CURRENT_SOURCE_DIR}/Replay/Main.cpp")
target_link_libraries(tautrope-replay PRIVATE tautrope_core)

# The persistent hardening: recordings get deleted once a fix lands, these stay.
# Deliberately dependency-free, so the whole standalone side is one cmake call.
file(GLOB TAUTROPE_TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/Tests/*.cpp")
add_executable(tautrope-tests ${TAUTROPE_TEST_SOURCES})
target_link_libraries(tautrope-tests PRIVATE tautrope_core)
target_include_directories(tautrope-tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/Tests")

enable_testing()
add_test(NAME core COMMAND tautrope-tests)
