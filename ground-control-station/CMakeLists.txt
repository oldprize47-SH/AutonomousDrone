cmake_minimum_required(VERSION 3.20)

project(GCS)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
# Dear ImGui (vendored in third_party/imgui) - Win32 + OpenGL3 backend
# Clone once with:
#   git clone --depth 1 --branch v1.91.5 https://github.com/ocornut/imgui.git third_party/imgui
# If the directory is missing, FetchContent fetches it at configure time.
# ============================================================
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui)

if(NOT EXISTS ${IMGUI_DIR}/imgui.cpp)
    include(FetchContent)
    FetchContent_Declare(
            imgui_dl
            GIT_REPOSITORY https://github.com/ocornut/imgui.git
            GIT_TAG        v1.91.5
            SOURCE_DIR     ${IMGUI_DIR}
    )
    FetchContent_MakeAvailable(imgui_dl)
endif()

add_library(imgui STATIC
        ${IMGUI_DIR}/imgui.cpp
        ${IMGUI_DIR}/imgui_draw.cpp
        ${IMGUI_DIR}/imgui_tables.cpp
        ${IMGUI_DIR}/imgui_widgets.cpp
        ${IMGUI_DIR}/backends/imgui_impl_win32.cpp
        ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC
        ${IMGUI_DIR}
        ${IMGUI_DIR}/backends
)

# ============================================================
# Main GCS
# ============================================================

add_executable(GCS
        gcs_main.cpp
        src/serial_port.cpp
        src/p8_comm.cpp
        src/ntrip_client.cpp
        src/RealTime.cpp
        src/OptiTrack.cpp
        src/dashboard.cpp
)

target_include_directories(GCS PRIVATE include)
target_link_libraries(GCS imgui)

if(WIN32)
    target_link_libraries(GCS ws2_32 opengl32 gdi32 dwmapi)
endif()

# ============================================================
# UDP telemetry dummy generator
# ============================================================

add_executable(UDP_test
        UDP_test.cpp
)

if(WIN32)
    target_link_libraries(UDP_test ws2_32)
endif()

# ============================================================
# OptiTrack standalone test
# ============================================================

add_executable(optitrack_test
        optitrack_test.cpp
        src/OptiTrack.cpp
)

target_include_directories(optitrack_test PRIVATE include)

if(WIN32)
    target_link_libraries(optitrack_test ws2_32)
endif()