#include "engine/gpu/DynamicGpuHashGrid.h"
#include <cuda_runtime.h>
#include <iostream>
#include <cmath>
#include <chrono>

namespace Urbaxio::Engine {

// ============================================================================
// DEVICE FUNCTIONS (GPU side)
// ============================================================================

__device__ inline uint32_t hash3D(int32_t x, int32_t y, int32_t z, uint32_t tableSize) {
    // Simple hash function
    uint32_t h = x * 73856093u ^ y * 19349663u ^ z * 83492791u;
    return h & (tableSize - 1); // Assumes tableSize is power of 2
}

__device__ inline int32_t voxelToBlock(float coord, float voxelSize) {
    return static_cast<int32_t>(floorf(coord / (voxelSize * DynamicGpuHashGrid::BLOCK_SIZE)));
}

__device__ inline int3 voxelToBlockCoord(float3 worldPos, float voxelSize) {
    return make_int3(
        voxelToBlock(worldPos.x, voxelSize),
        voxelToBlock(worldPos.y, voxelSize),
        voxelToBlock(worldPos.z, voxelSize)
    );
}

// Find or allocate block in hash table
// Returns dataIndex (UINT32_MAX if allocation failed)
__device__ uint32_t findOrAllocateBlock(
    DynamicGpuHashGrid::HashEntry* hashTable,
    uint32_t tableSize,
    int32_t blockX, int32_t blockY, int32_t blockZ,
    uint32_t* blockCounter,
    uint32_t maxBlocks,
    float* blockData)
{
    uint32_t slot = hash3D(blockX, blockY, blockZ, tableSize);

    // Linear probing
    for (uint32_t probe = 0; probe < 100; ++probe) {
        uint32_t currentSlot = (slot + probe) & (tableSize - 1);

        // Try to claim this slot
        int32_t oldX = atomicCAS(&hashTable[currentSlot].blockX, INT32_MAX, blockX);

        if (oldX == INT32_MAX) {
            // We claimed an empty slot! Initialize it
            hashTable[currentSlot].blockY = blockY;
            hashTable[currentSlot].blockZ = blockZ;

            // Allocate block data
            uint32_t dataIdx = atomicAdd(blockCounter, 1);

            if (dataIdx >= maxBlocks) {
                // Out of memory! Mark slot as failed
                hashTable[currentSlot].blockX = INT32_MAX;
                return UINT32_MAX;
            }

            hashTable[currentSlot].dataIndex = dataIdx;

            // Initialize block with background value (outside = 1.0)
            float* block = blockData + dataIdx * DynamicGpuHashGrid::VOXELS_PER_BLOCK;
            for (int i = threadIdx.x; i < DynamicGpuHashGrid::VOXELS_PER_BLOCK; i += blockDim.x) {
                block[i] = 1.0f;
            }
            __syncthreads();

            return dataIdx;
        }

        // Check if this slot has our block
        if (hashTable[currentSlot].blockX == blockX &&
            hashTable[currentSlot].blockY == blockY &&
            hashTable[currentSlot].blockZ == blockZ) {
            return hashTable[currentSlot].dataIndex;
        }
    }

    // Hash table collision limit reached
    return UINT32_MAX;
}

// Sculpt kernel - processes one voxel per thread
__global__ void SculptSphericalKernel(
    DynamicGpuHashGrid::HashEntry* hashTable,
    uint32_t tableSize,
    float* blockData,
    uint32_t* blockCounter,
    uint32_t maxBlocks,
    float3 brushCenter,
    float brushRadius,
    float strength,
    bool addMode,
    float voxelSize)
{
    // Each block processes one potential block coordinate
    // Use floorf for consistent coordinate system with meshing
    int32_t centerBlockX = static_cast<int32_t>(floorf(brushCenter.x / (voxelSize * DynamicGpuHashGrid::BLOCK_SIZE)));
    int32_t centerBlockY = static_cast<int32_t>(floorf(brushCenter.y / (voxelSize * DynamicGpuHashGrid::BLOCK_SIZE)));
    int32_t centerBlockZ = static_cast<int32_t>(floorf(brushCenter.z / (voxelSize * DynamicGpuHashGrid::BLOCK_SIZE)));
    
    int32_t blockX = centerBlockX + blockIdx.x - gridDim.x / 2;
    int32_t blockY = centerBlockY + blockIdx.y - gridDim.y / 2;
    int32_t blockZ = centerBlockZ + blockIdx.z - gridDim.z / 2;

    // Find or allocate this block
    uint32_t dataIdx = findOrAllocateBlock(
        hashTable, tableSize,
        blockX, blockY, blockZ,
        blockCounter, maxBlocks, blockData
    );

    if (dataIdx == UINT32_MAX) return; // Allocation failed

    // Each thread processes one voxel in the block
    int voxelIdx = threadIdx.x;
    if (voxelIdx >= DynamicGpuHashGrid::VOXELS_PER_BLOCK) return;

    // Decode voxel coordinates within block
    int localX = voxelIdx % DynamicGpuHashGrid::BLOCK_SIZE;
    int localY = (voxelIdx / DynamicGpuHashGrid::BLOCK_SIZE) % DynamicGpuHashGrid::BLOCK_SIZE;
    int localZ = voxelIdx / (DynamicGpuHashGrid::BLOCK_SIZE * DynamicGpuHashGrid::BLOCK_SIZE);

    // World position of this voxel
    float3 voxelWorld = make_float3(
        (blockX * DynamicGpuHashGrid::BLOCK_SIZE + localX) * voxelSize,
        (blockY * DynamicGpuHashGrid::BLOCK_SIZE + localY) * voxelSize,
        (blockZ * DynamicGpuHashGrid::BLOCK_SIZE + localZ) * voxelSize
    );

    // Distance from brush center
    float dx = voxelWorld.x - brushCenter.x;
    float dy = voxelWorld.y - brushCenter.y;
    float dz = voxelWorld.z - brushCenter.z;
    float dist = sqrtf(dx*dx + dy*dy + dz*dz);

    if (dist > brushRadius + voxelSize * 2.0f) return; // Outside brush influence

    // Brush SDF
    float brushSDF = dist - brushRadius;

    // Get current value
    float* block = blockData + dataIdx * DynamicGpuHashGrid::VOXELS_PER_BLOCK;
    float currentVal = block[voxelIdx];

    // CSG union (smooth min)
    const float smoothing = 0.5f;
    float h = fmaxf(smoothing - fabsf(currentVal - brushSDF), 0.0f) / smoothing;
    float newVal = fminf(currentVal, brushSDF) - h * h * smoothing * 0.25f;

    // Falloff
    float falloff = 1.0f - (dist / (brushRadius + voxelSize * 2.0f));
    falloff = fmaxf(0.0f, fminf(1.0f, falloff));
    falloff = falloff * falloff;

    float blended = currentVal + (newVal - currentVal) * strength * falloff;

    // Write back
    block[voxelIdx] = blended;
}

// ============================================================================
// HOST FUNCTIONS (CPU side)
// ============================================================================

DynamicGpuHashGrid::DynamicGpuHashGrid(const Config& config)
    : config_(config)
    , d_hashTable_(nullptr)
    , d_blockData_(nullptr)
    , d_blockCounter_(nullptr)
    , activeBlockCount_(0)
{
    initialize();
}

DynamicGpuHashGrid::~DynamicGpuHashGrid() {
    cleanup();
}

void DynamicGpuHashGrid::initialize() {
    std::cout << "[DynamicGpuHashGrid] Initializing..." << std::endl;
    
    size_t freeMemory, totalMemory;
    cudaMemGetInfo(&freeMemory, &totalMemory);
    float freeGB = freeMemory / (1024.0f * 1024.0f * 1024.0f);
    
    std::cout << "  Available GPU memory: " << freeGB << " GB" << std::endl;
    
    if (freeGB < 2.0f) {
        std::cout << "[DynamicGpuHashGrid] ⚠️ Low GPU memory (" << freeGB 
                  << " GB), reducing allocation..." << std::endl;
        config_.maxBlocks = std::min(config_.maxBlocks, 100u * 1024u);
        config_.hashTableSize = std::min(config_.hashTableSize, 256u * 1024u);
    }
    
    std::cout << "  Max blocks: " << config_.maxBlocks << std::endl;
    std::cout << "  Hash table size: " << config_.hashTableSize << std::endl;
    std::cout << "  Voxel size: " << config_.voxelSize << "m" << std::endl;

    size_t hashTableBytes = config_.hashTableSize * sizeof(HashEntry);
    size_t blockDataBytes = config_.maxBlocks * VOXELS_PER_BLOCK * sizeof(float);

    std::cout << "  Memory required: " << (hashTableBytes + blockDataBytes) / (1024.0f * 1024.0f) << " MB" << std::endl;

    // Allocate hash table
    cudaMalloc(&d_hashTable_, hashTableBytes);

    // Initialize hash table to empty (blockX = INT32_MAX means empty)
    std::vector<HashEntry> emptyTable(config_.hashTableSize);
    for (auto& entry : emptyTable) {
        entry.blockX = INT32_MAX;
        entry.blockY = 0;
        entry.blockZ = 0;
        entry.dataIndex = 0;
    }
    cudaMemcpy(d_hashTable_, emptyTable.data(), hashTableBytes, cudaMemcpyHostToDevice);

    // Allocate block data
    cudaMalloc(&d_blockData_, blockDataBytes);

    // Allocate block counter
    cudaMalloc(&d_blockCounter_, sizeof(uint32_t));
    cudaMemset(d_blockCounter_, 0, sizeof(uint32_t));

    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        std::cerr << "[DynamicGpuHashGrid] CUDA error: " << cudaGetErrorString(err) << std::endl;
        return;
    }
    
    std::cout << "[DynamicGpuHashGrid] ✅ Initialized successfully!" << std::endl;
}

void DynamicGpuHashGrid::cleanup() {
    if (d_hashTable_) cudaFree(d_hashTable_);
    if (d_blockData_) cudaFree(d_blockData_);
    if (d_blockCounter_) cudaFree(d_blockCounter_);
    if (d_meshVertices_) cudaFree(d_meshVertices_);
    if (d_meshNormals_) cudaFree(d_meshNormals_);
}

bool DynamicGpuHashGrid::ApplySphericalBrush(
    const glm::vec3& worldPos,
    float worldRadius,
    float strength,
    bool addMode,
    void* stream)
{
    // Calculate affected blocks
    int blockRadius = static_cast<int>(ceilf(worldRadius / (config_.voxelSize * BLOCK_SIZE))) + 1;

    dim3 gridSize(blockRadius * 2 + 1, blockRadius * 2 + 1, blockRadius * 2 + 1);
    dim3 blockSize(VOXELS_PER_BLOCK);

    float3 brushCenter = make_float3(worldPos.x, worldPos.y, worldPos.z);

    cudaStream_t cudaStream = stream ? static_cast<cudaStream_t>(stream) : 0;

    SculptSphericalKernel<<<gridSize, blockSize, 0, cudaStream>>>(
        d_hashTable_,
        config_.hashTableSize,
        d_blockData_,
        d_blockCounter_,
        config_.maxBlocks,
        brushCenter,
        worldRadius,
        strength,
        addMode,
        config_.voxelSize
    );

    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        std::cerr << "[DynamicGpuHashGrid] Sculpt kernel error: " << cudaGetErrorString(err) << std::endl;
        return false;
    }
    
    // ✅ Mark count as dirty (don't fetch yet!)
    needsCountUpdate_ = true;

    return true;
}

uint32_t DynamicGpuHashGrid::GetActiveBlockCount() const {
    // ✅ Return cached value (update ASYNC in background)
    return cachedActiveBlockCount_;
}

void DynamicGpuHashGrid::UpdateActiveCountAsync(void* stream) {
    if (!needsCountUpdate_) return;
    
    cudaStream_t cudaStream = stream ? static_cast<cudaStream_t>(stream) : 0;
    
    // Non-blocking copy
    cudaMemcpyAsync(&cachedActiveBlockCount_, d_blockCounter_, 
                    sizeof(uint32_t), cudaMemcpyDeviceToHost, cudaStream);
    
    needsCountUpdate_ = false;
}

void DynamicGpuHashGrid::PrintStats() const {
    uint32_t activeBlocks = GetActiveBlockCount();
    float usedMemoryMB = activeBlocks * VOXELS_PER_BLOCK * sizeof(float) / (1024.0f * 1024.0f);
    float fillRate = 100.0f * activeBlocks / config_.maxBlocks;

    std::cout << "[DynamicGpuHashGrid] Stats:" << std::endl;
    std::cout << "  Active blocks: " << activeBlocks << " / " << config_.maxBlocks 
              << " (" << fillRate << "%)" << std::endl;
    std::cout << "  Used memory: " << usedMemoryMB << " MB" << std::endl;
}

// Forward declaration - implementation in HashGridMeshing.cu
// (ExtractMesh is defined in a separate file)

} // namespace Urbaxio::Engine


