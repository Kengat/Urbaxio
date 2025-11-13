# GPU Acceleration Module (Path B - CUDA/NanoVDB)
# This module is only built if CUDA is available

message(STATUS "Configuring GPU acceleration module...")

# Find CUDA Toolkit
find_package(CUDAToolkit REQUIRED)
message(STATUS "CUDA Toolkit found: ${CUDAToolkit_VERSION}")

# GPU module sources
set(GPU_SOURCES
    GpuVoxelManager.cu
    GpuSculptKernels.cu
    GpuMeshingKernels.cu
    GpuRaycastKernels.cu  # NEW
    GpuAabbKernels.cu
    MeshManagerGpu.cu
    GpuVoxelConverter.cpp  # CPU-side converter (NOT compiled with nvcc)
)

# Create static library for GPU code
add_library(urbaxio_gpu STATIC ${GPU_SOURCES})

# Set CUDA properties - use single architecture to avoid MSBuild issues
set_target_properties(urbaxio_gpu PROPERTIES
    CUDA_SEPARABLE_COMPILATION OFF
    CUDA_RESOLVE_DEVICE_SYMBOLS OFF
    CUDA_STANDARD 17
    CUDA_ARCHITECTURES "75"  # Simplified to one arch to avoid MSBuild command line issues
    LINKER_LANGUAGE CXX
)

# Include directories
target_include_directories(urbaxio_gpu PUBLIC
    ${CMAKE_SOURCE_DIR}/engine/include
    ${CMAKE_SOURCE_DIR}/cad_kernel/include
)

# Find dependencies from vcpkg
find_package(OpenVDB CONFIG REQUIRED)
find_package(TBB CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)

# Link libraries
target_link_libraries(urbaxio_gpu PUBLIC
    CUDA::cudart
    CUDA::cuda_driver
    OpenVDB::openvdb
    TBB::tbb
    glm::glm
)

# CUDA compiler flags - minimal to avoid MSBuild issues  
# Use response files for long command lines
set_target_properties(urbaxio_gpu PROPERTIES
    CUDA_USE_RESPONSE_FILE_FOR_INCLUDES ON
    CUDA_USE_RESPONSE_FILE_FOR_LIBRARIES ON
    CUDA_USE_RESPONSE_FILE_FOR_OBJECTS ON
)

target_compile_options(urbaxio_gpu PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:
        --expt-relaxed-constexpr
        --expt-extended-lambda
        -Xcompiler=/bigobj
        -Xcompiler=/wd4819
        -DOPENVDB_USE_DELAYED_LOADING
        -DOPENVDB_STATICLIB
        --diag-suppress=20054
        --diag-suppress=20012
    >
    $<$<COMPILE_LANGUAGE:CXX>:
        /bigobj
        -DNANOVDB_USE_OPENVDB
    >
)

# Make GPU library available to other targets
set_target_properties(urbaxio_gpu PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

message(STATUS "GPU module configured successfully")
message(STATUS "  CUDA Architectures: 75 (RTX 20xx), 86 (RTX 30xx), 89 (RTX 40xx)")
message(STATUS "  NanoVDB support: via OpenVDB")

