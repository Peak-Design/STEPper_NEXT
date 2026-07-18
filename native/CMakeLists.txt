cmake_minimum_required(VERSION 3.20)
project(stepper_native LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ── Find Python (must match Blender's embedded Python 3.13) ──────────
find_package(Python3 3.13 EXACT REQUIRED COMPONENTS Development NumPy)

# ── OpenCascade: skip find_package, use OCCT_ROOT directly ──────────
# The OCCT CMake config has cross-component deps (VTK, etc.) that
# break when only a subset is installed.  We only need headers + a
# few libs, so we locate them manually via OCCT_ROOT.
if(NOT DEFINED OCCT_ROOT)
    # Default: assume conda environment layout
    if(WIN32)
        set(OCCT_ROOT "$ENV{CONDA_PREFIX}/Library")
    else()
        set(OCCT_ROOT "$ENV{CONDA_PREFIX}")
    endif()
endif()

set(OCCT_INCLUDE_DIR "${OCCT_ROOT}/include/opencascade")
if(NOT EXISTS "${OCCT_INCLUDE_DIR}")
    message(FATAL_ERROR "OCCT headers not found at ${OCCT_INCLUDE_DIR}")
endif()

if(WIN32)
    set(OCCT_LIB_DIR "${OCCT_ROOT}/lib")
else()
    set(OCCT_LIB_DIR "${OCCT_ROOT}/lib")
endif()

# ── Build the extension module ───────────────────────────────────────
Python3_add_library(stepper_native MODULE stepper_native.cpp)

target_include_directories(stepper_native PRIVATE
    ${Python3_INCLUDE_DIRS}
    ${Python3_NumPy_INCLUDE_DIRS}
    ${OCCT_INCLUDE_DIR}
)

target_link_directories(stepper_native PRIVATE ${OCCT_LIB_DIR})

target_link_libraries(stepper_native PRIVATE
    TKernel TKMath TKBRep TKTopAlgo TKG3d TKMesh
)

# Remove "lib" prefix on all platforms (Python expects bare module name)
set_target_properties(stepper_native PROPERTIES
    PREFIX ""
)

# On Windows, set the output suffix to .pyd
if(WIN32)
    set_target_properties(stepper_native PROPERTIES
        SUFFIX ".pyd"
    )
endif()
