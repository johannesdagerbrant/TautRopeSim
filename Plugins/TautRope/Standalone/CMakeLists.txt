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
project(TautRopeStandalone CXX)

set(CMAKE_CXX_STANDARD 17)
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

# Floating-point settings must match what UBT gives the editor build, or a
# recording will not replay bit-identically. See issue #12; these are the
# starting point, not the finished answer.
if(MSVC)
	target_compile_options(tautrope_core PUBLIC /fp:precise /fp:except-)
	target_compile_options(tautrope_core PRIVATE /W4 /permissive-)
	# Deliberately no /arch: flag. The x64 default is SSE2, and letting the
	# compiler auto-vectorise with AVX would diverge from the editor build.
else()
	target_compile_options(tautrope_core PUBLIC -ffp-contract=off)
	target_compile_options(tautrope_core PRIVATE -Wall -Wextra)
endif()

add_executable(tautrope-replay "${CMAKE_CURRENT_SOURCE_DIR}/Replay/Main.cpp")
target_link_libraries(tautrope-replay PRIVATE tautrope_core)
