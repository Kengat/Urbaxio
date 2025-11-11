
#pragma once



#include <glm/glm.hpp>

#include <vector>



namespace Urbaxio::Engine {



class GpuSculptKernels {
public:
    enum class SculptMode {
        ADD = 0,
        SUBTRACT = 1
    };



    // Primary sculpting function: Direct NanoVDB modification on GPU
    // Modifies values within existing topology (no new voxels can be added)
    // brushCenter: Position in INDEX/VOXEL space (NOT world space)
    // brushRadius: Radius in VOXEL units (NOT world units)
    // leafCount: Number of leaf nodes (must be obtained on CPU before upload)
    // Returns: true if successful, false on error
    static bool ApplySphericalBrush(
        void* deviceGridPtr,           // NanoVDB grid on GPU
        uint64_t leafCount,            // Number of leaf nodes (from CPU metadata)
        const glm::vec3& brushCenter,  // In INDEX space
        float brushRadius,             // In VOXEL units
        float voxelSize,               // For reference only
        SculptMode mode,
        float strength = 1.0f,
        const glm::ivec3& gridBBoxMin = glm::ivec3(0), // Current grid bounds (for UI/debugging)
        const glm::ivec3& gridBBoxMax = glm::ivec3(0),
        std::vector<float>* outModifiedBuffer = nullptr,  // Unused in optimized path
        glm::ivec3* outMinVoxel = nullptr,
        glm::ivec3* outMaxVoxel = nullptr,
        void* cudaStream = nullptr     // CUDA stream for async execution
    );



    // Legacy functions (kept for API compatibility, not used in optimized path)
    static bool CreateBrushGrid(
        const glm::vec3& brushCenter,
        float brushRadius,
        float voxelSize,
        void** outDevicePtr,
        size_t* outSizeBytes
    );



    static bool CsgUnion(void* sceneGridPtr, void* brushGridPtr);
    static bool CsgDifference(void* sceneGridPtr, void* brushGridPtr);
    static void FreeDeviceGrid(void* devicePtr);
};



} // namespace Urbaxio::Engine

