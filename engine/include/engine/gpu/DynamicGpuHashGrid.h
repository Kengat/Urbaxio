#pragma once

#include <glm/glm.hpp>
#include <cstdint>
#include <atomic>

namespace Urbaxio::Engine {

class DynamicGpuHashGrid {
public:
    static constexpr int BLOCK_SIZE = 8;
    static constexpr int VOXELS_PER_BLOCK = 512; // 8^3
    static constexpr int MAX_TRIANGLES_PER_BLOCK = 80; // Worst case ~5 per voxel face

    struct Config {
        float voxelSize = 0.05f;
        uint32_t maxBlocks = 200 * 1024;
        uint32_t hashTableSize = 512 * 1024;
        uint32_t maxDirtyBlocks = 8192;      // NEW: Max dirty blocks per frame
        bool enableIncrementalMesh = true;    // NEW: Enable incremental updates
    };

    struct HashEntry {
        int32_t blockX;
        int32_t blockY;
        int32_t blockZ;
        uint32_t dataIndex;
    };

    // NEW: Per-block mesh info for incremental updates
    struct BlockMeshInfo {
        uint32_t triangleOffset;  // Where this block's triangles start
        uint32_t triangleCount;   // How many triangles this block has
        uint32_t lastUpdateFrame; // Frame when last updated
    };

    // NEW: Mesh statistics
    struct MeshStats {
        uint32_t totalTriangles;
        uint32_t dirtyBlockCount;
        uint32_t activeBlockCount;
        float lastExtractTimeMs;
        float lastIncrementalTimeMs;
    };

    DynamicGpuHashGrid(const Config& config);
    ~DynamicGpuHashGrid();

    // Sculpting
    bool ApplySphericalBrush(
        const glm::vec3& worldPos,
        float worldRadius,
        float strength,
        bool addMode,
        void* stream = nullptr
    );

    // NEW: Optimized mesh extraction with incremental support
    bool ExtractMesh(
        float isovalue,
        float** d_vertices,
        float** d_normals,
        int** d_triangleCounter,
        void* stream = nullptr
    );

    // NEW: Force full mesh rebuild (call after major changes)
    void InvalidateAllBlocks();

    // NEW: Get current mesh without re-extracting
    bool GetCurrentMesh(
        float** d_vertices,
        float** d_normals,
        int* triangleCount
    );

    void Reset();
    void PrintStats() const;

    uint32_t GetActiveBlockCount() const;
    uint32_t GetMaxBlocks() const { return config_.maxBlocks; }
    float GetVoxelSize() const { return config_.voxelSize; }

    void UpdateActiveCountAsync(void* stream = nullptr);

    // NEW: Statistics
    MeshStats GetMeshStats() const;
    uint32_t GetDirtyBlockCount() const;

private:
    void initialize();
    void cleanup();
    void initializeMeshBuffers();
    void cleanupMeshBuffers();

    // NEW: Incremental mesh helpers
    bool extractMeshFull(float isovalue, void* stream);
    bool extractMeshIncremental(float isovalue, void* stream);
    void compactActiveBlocks(void* stream);
    void compactDirtyBlocks(void* stream);
    void clearDirtyList(void* stream);
    void smoothNormals(int vertexCount, void* stream);

    Config config_;

    // Hash table
    HashEntry* d_hashTable_ = nullptr;
    float* d_blockData_ = nullptr;
    uint32_t* d_blockCounter_ = nullptr;

    // Active block tracking
    uint32_t* d_activeIndices_ = nullptr;
    uint32_t* d_activeCount_ = nullptr;
    uint32_t cachedActiveBlockCount_ = 0;
    bool needsCountUpdate_ = true;

    // NEW: Dirty block tracking
    uint32_t* d_dirtyBlockIndices_ = nullptr;
    uint32_t* d_dirtyCount_ = nullptr;
    uint32_t* d_dirtyFlags_ = nullptr;  // Per-block dirty flag
    uint32_t cachedDirtyCount_ = 0;

    // NEW: Per-block mesh info
    BlockMeshInfo* d_blockMeshInfo_ = nullptr;

    // NEW: Double-buffered mesh output
    struct MeshBuffer {
        float* d_vertices = nullptr;
        float* d_normals = nullptr;
        int triangleCount = 0;
        bool valid = false;
    };
    MeshBuffer meshBuffers_[2];
    int currentMeshBuffer_ = 0;
    uint32_t allocatedTriangles_ = 0;

    // NEW: Temporary buffers for incremental updates
    float* d_tempVertices_ = nullptr;
    float* d_tempNormals_ = nullptr;
    int* d_tempTriangleCount_ = nullptr;
    uint32_t tempBufferSize_ = 0;

    // NEW: Normal smoothing hash table (persistent)
    float* d_normalAccumTable_ = nullptr;
    uint32_t normalTableSize_ = 0;

    // Statistics
    uint32_t frameCounter_ = 0;
    mutable MeshStats lastStats_ = {};

    // Legacy compatibility
    float* d_meshVertices_ = nullptr;
    float* d_meshNormals_ = nullptr;
};

} // namespace Urbaxio::Engine