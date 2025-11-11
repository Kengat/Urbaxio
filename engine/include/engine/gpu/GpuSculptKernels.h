#pragma once

#include <cstdint>
#include <glm/glm.hpp>

namespace Urbaxio::Engine {

/**
 * @brief GPU kernels for volumetric sculpting operations
 * 
 * These kernels perform CSG-like operations directly on NanoVDB grids in GPU memory.
 * All operations are performed in parallel across GPU threads.
 */
class GpuSculptKernels {
public:
    enum class SculptMode {
        ADD,        // Add material (CSG Union)
        SUBTRACT,   // Remove material (CSG Difference)
        SMOOTH      // Smooth surface
    };

    /**
     * @brief Apply a spherical brush to a NanoVDB grid on GPU
     * 
     * @param deviceGridPtr Device pointer to the NanoVDB grid
     * @param brushCenter World-space position of the brush center
     * @param brushRadius Radius of the brush in world units
     * @param voxelSize Size of a voxel in world units
     * @param mode Sculpting mode (add/subtract/smooth)
     * @param strength Brush strength (0.0 to 1.0)
     * @return True if successful
     */
    static bool ApplySphericalBrush(
        void* deviceGridPtr,
        const glm::vec3& brushCenter,
        float brushRadius,
        float voxelSize,
        SculptMode mode,
        float strength = 1.0f
    );

    /**
     * @brief Create a temporary brush grid on GPU
     * 
     * @param brushCenter Center of the brush in world space
     * @param brushRadius Radius in world units
     * @param voxelSize Voxel size in world units
     * @param outDevicePtr Output device pointer to the created brush grid
     * @param outSizeBytes Output size of the created grid in bytes
     * @return True if successful
     */
    static bool CreateBrushGrid(
        const glm::vec3& brushCenter,
        float brushRadius,
        float voxelSize,
        void** outDevicePtr,
        size_t* outSizeBytes
    );

    /**
     * @brief Perform CSG union between two grids on GPU
     * 
     * @param sceneGridPtr Device pointer to the scene grid (modified in-place)
     * @param brushGridPtr Device pointer to the brush grid (read-only)
     * @return True if successful
     */
    static bool CsgUnion(void* sceneGridPtr, void* brushGridPtr);

    /**
     * @brief Perform CSG difference between two grids on GPU
     * 
     * @param sceneGridPtr Device pointer to the scene grid (modified in-place)
     * @param brushGridPtr Device pointer to the brush grid (read-only)
     * @return True if successful
     */
    static bool CsgDifference(void* sceneGridPtr, void* brushGridPtr);

    /**
     * @brief Free a temporary GPU grid
     * 
     * @param devicePtr Device pointer to free
     */
    static void FreeDeviceGrid(void* devicePtr);
};

} // namespace Urbaxio::Engine

