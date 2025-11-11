#pragma once

#include <cstdint>
#include <vector>
#include "cad_kernel/cad_kernel.h"

namespace Urbaxio::Engine {

/**
 * @brief GPU-accelerated meshing kernels
 * 
 * Implements Marching Cubes algorithm on GPU for converting
 * volumetric NanoVDB grids to triangle meshes.
 */
class GpuMeshingKernels {
public:
    /**
     * @brief Generate a mesh from a NanoVDB grid on GPU using Marching Cubes
     * 
     * @param deviceGridPtr Device pointer to the NanoVDB grid
     * @param isoValue ISO surface value (typically 0.0 for SDF)
     * @param voxelSize Size of a voxel in world units
     * @param outMesh Output mesh buffers
     * @return True if successful
     */
    static bool MarchingCubes(
        void* deviceGridPtr,
        float isoValue,
        float voxelSize,
        CadKernel::MeshBuffers& outMesh
    );

    /**
     * @brief Check if GPU meshing is available
     */
    static bool IsAvailable();

    /**
     * @brief Estimate output triangle count (for memory allocation)
     */
    static size_t EstimateTriangleCount(void* deviceGridPtr, float isoValue);

private:
    // Marching Cubes lookup tables (stored in GPU constant memory)
    static void InitializeLookupTables();
};

} // namespace Urbaxio::Engine

