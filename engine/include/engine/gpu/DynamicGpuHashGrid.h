#pragma once

#include <glm/glm.hpp>
#include <cstdint>
#include <vector>

namespace Urbaxio::Engine {

// GPU Hash-based Sparse Voxel Grid with dynamic allocation
// Unlimited range, full GPU speed, zero compromises
class DynamicGpuHashGrid {
public:
    // Block is 8x8x8 voxels (512 floats = 2KB)
    static constexpr int BLOCK_SIZE = 8;
    static constexpr int VOXELS_PER_BLOCK = BLOCK_SIZE * BLOCK_SIZE * BLOCK_SIZE;

    struct Config {
        float voxelSize = 0.05f;           // 5cm voxels
        uint32_t maxBlocks = 1024 * 1024;  // 1M blocks = 512MB = ~400m³ at 5cm voxels
        uint32_t hashTableSize = 2097152;  // 2M entries (must be power of 2)
    };

    // Hash table entry - exposed for CUDA kernels
    struct HashEntry {
        int32_t blockX, blockY, blockZ;  // Block coordinates
        uint32_t dataIndex;               // Index into blockData array
        uint32_t padding;                 // Keep 32-byte alignment
    };

    DynamicGpuHashGrid(const Config& config);
    ~DynamicGpuHashGrid();

    // Sculpting operations
    bool ApplySphericalBrush(
        const glm::vec3& worldPos,
        float worldRadius,
        float strength,
        bool addMode,
        void* stream = nullptr
    );

    // Meshing
    bool ExtractMesh(
        float isovalue,
        float** d_vertices,    // Output: device pointer to vertices
        float** d_normals,     // Output: device pointer to normals
        int** d_triangleCounter_out, // Output: device pointer to triangle counter
        void* stream = nullptr
    );

    // Stats
    uint32_t GetActiveBlockCount() const;
    uint32_t GetMaxBlocks() const { return config_.maxBlocks; }
    float GetVoxelSize() const { return config_.voxelSize; }

    // Debug
    void PrintStats() const;

private:
    Config config_;

    // GPU data structures

    HashEntry* d_hashTable_;              // GPU hash table
    float* d_blockData_;                  // GPU block data (all voxels)
    uint32_t* d_blockCounter_;            // GPU atomic counter for allocation

    // Persistent mesh buffers (memory pool)
    float* d_meshVertices_ = nullptr;
    float* d_meshNormals_ = nullptr;
    size_t allocatedTriangles_ = 0;

    // CPU tracking
    uint32_t activeBlockCount_;

    // Helper functions
    void initialize();
    void cleanup();
};

} // namespace Urbaxio::Engine


