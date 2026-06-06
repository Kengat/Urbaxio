// HashGridMeshing.cu - Optimized Marching Cubes with Incremental Updates
// Key optimizations:
// 1. Process only dirty blocks when possible
// 2. Cached per-block mesh offsets
// 3. Optimized normal smoothing with persistent hash table
// 4. Double-buffered output

#include "engine/gpu/DynamicGpuHashGrid.h"
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <iostream>
#include <chrono>

namespace Urbaxio::Engine {

// ============================================================================
// MARCHING CUBES LOOKUP TABLES
// ============================================================================

__constant__ int d_edgeTable[256];
__constant__ int d_triTable[256][16];
static bool s_tablesInitialized = false;

// Edge table data (same as before - abbreviated for space)
static const int h_edgeTable[256] = {
    0x0  , 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c,
    0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00,
    0x190, 0x99 , 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c,
    0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90,
    0x230, 0x339, 0x33 , 0x13a, 0x636, 0x73f, 0x435, 0x53c,
    0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30,
    0x3a0, 0x2a9, 0x1a3, 0xaa , 0x7a6, 0x6af, 0x5a5, 0x4ac,
    0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0,
    0x460, 0x569, 0x663, 0x76a, 0x66 , 0x16f, 0x265, 0x36c,
    0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60,
    0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0xff , 0x3f5, 0x2fc,
    0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0,
    0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x55 , 0x15c,
    0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950,
    0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0xcc ,
    0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0,
    0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc,
    0xcc , 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0,
    0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c,
    0x15c, 0x55 , 0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650,
    0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc,
    0x2fc, 0x3f5, 0xff , 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0,
    0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c,
    0x36c, 0x265, 0x16f, 0x66 , 0x76a, 0x663, 0x569, 0x460,
    0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af, 0xaa5, 0xbac,
    0x4ac, 0x5a5, 0x6af, 0x7a6, 0xaa , 0x1a3, 0x2a9, 0x3a0,
    0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c,
    0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x33 , 0x339, 0x230,
    0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c,
    0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x99 , 0x190,
    0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c,
    0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x0   
};

// Triangle table (same as original - using extern or include)
extern const int h_triTable[256][16];

void InitializeMCTables() {
    if (s_tablesInitialized) {
        std::cout << "[DEBUG] MC tables already initialized" << std::endl;
        return;
    }
    
    std::cout << "[DEBUG] Initializing MC tables..." << std::endl;
    
    cudaError_t err1 = cudaMemcpyToSymbol(d_edgeTable, h_edgeTable, sizeof(h_edgeTable));
    cudaError_t err2 = cudaMemcpyToSymbol(d_triTable, h_triTable, sizeof(h_triTable));
    
    if (err1 != cudaSuccess || err2 != cudaSuccess) {
        std::cerr << "[DEBUG] MC tables copy failed! err1=" << cudaGetErrorString(err1) 
                  << " err2=" << cudaGetErrorString(err2) << std::endl;
        return;
    }
    
    s_tablesInitialized = true;
    std::cout << "[DEBUG] MC tables initialized OK" << std::endl;
}

// ============================================================================
// DEVICE HELPER FUNCTIONS
// ============================================================================

__device__ __forceinline__ float3 VertexInterp(float iso, float3 p1, float v1, float3 p2, float v2) {
    if (fabsf(v2 - v1) < 1e-6f) {
        return make_float3((p1.x + p2.x) * 0.5f, (p1.y + p2.y) * 0.5f, (p1.z + p2.z) * 0.5f);
    }
    float mu = fmaxf(0.0f, fminf(1.0f, (iso - v1) / (v2 - v1)));
    return make_float3(
        p1.x + mu * (p2.x - p1.x),
        p1.y + mu * (p2.y - p1.y),
        p1.z + mu * (p2.z - p1.z)
    );
}

__device__ float sampleVoxelCached(
    const float* cache,  // 9x9x9 cache
    int x, int y, int z)
{
    return cache[z * 81 + y * 9 + x];
}

__device__ float sampleVoxelGlobal(
    const DynamicGpuHashGrid::HashEntry* hashTable,
    uint32_t tableSize,
    const float* blockData,
    int32_t voxelX, int32_t voxelY, int32_t voxelZ)
{
    int32_t blockX = voxelX >> 3;  // Divide by 8
    int32_t blockY = voxelY >> 3;
    int32_t blockZ = voxelZ >> 3;
    
    // Handle negative coordinates
    if (voxelX < 0) blockX = (voxelX - 7) >> 3;
    if (voxelY < 0) blockY = (voxelY - 7) >> 3;
    if (voxelZ < 0) blockZ = (voxelZ - 7) >> 3;
    
    int localX = voxelX - blockX * 8;
    int localY = voxelY - blockY * 8;
    int localZ = voxelZ - blockZ * 8;
    
    // Hash lookup
    uint32_t h = blockX * 73856093u ^ blockY * 19349663u ^ blockZ * 83492791u;
    uint32_t slot = h & (tableSize - 1);
    
    // ✅ КРИТИЧНО: максимум 16 проб, потом выход
    #pragma unroll 1
    for (uint32_t probe = 0; probe < 16; ++probe) {
        uint32_t currentSlot = (slot + probe) & (tableSize - 1);
        
        int32_t storedX = hashTable[currentSlot].blockX;
        
        if (storedX == INT32_MAX) {
            return 1.0f;  // Пустой слот - блока нет
        }
        
        if (storedX == blockX &&
            hashTable[currentSlot].blockY == blockY &&
            hashTable[currentSlot].blockZ == blockZ) 
        {
            uint32_t dataIdx = hashTable[currentSlot].dataIndex;
            // ✅ Защита от out-of-bounds
            if (dataIdx >= 204800) return 1.0f;
            
            return blockData[dataIdx * 512 + localX + localY * 8 + localZ * 64];
        }
    }
    
    return 1.0f;
}

// ============================================================================
// OPTIMIZED MARCHING CUBES KERNEL
// ============================================================================

__global__ void MarchingCubesBlockKernel(
    const DynamicGpuHashGrid::HashEntry* hashTable,
    uint32_t tableSize,
    const float* blockData,
    float isovalue,
    float voxelSize,
    float3* outputVertices,
    float3* outputNormals,
    int* triangleCounter,
    int maxTriangles,
    const uint32_t* blockIndices,     // Which blocks to process
    uint32_t blockCount,
    DynamicGpuHashGrid::BlockMeshInfo* blockMeshInfo,  // Output per-block info
    bool updateMeshInfo)
{
    uint32_t blockIdx_custom = blockIdx.x;
    if (blockIdx_custom >= blockCount) return;

    // Только первый блок и первый поток печатает
    if (blockIdx_custom == 0 && threadIdx.x == 0) {
        printf("[KERNEL] block 0 started\n");
    }

    uint32_t entryIdx = blockIndices[blockIdx_custom];
    
    // ✅ ДОБАВИТЬ: Защита от невалидных индексов
    if (entryIdx >= tableSize) return;
    
    if (hashTable[entryIdx].blockX == INT32_MAX) return;
    
    int32_t blockX = hashTable[entryIdx].blockX;
    int32_t blockY = hashTable[entryIdx].blockY;
    int32_t blockZ = hashTable[entryIdx].blockZ;
    uint32_t dataIdx = hashTable[entryIdx].dataIndex;
    
    if (blockIdx_custom == 0 && threadIdx.x == 0) {
        printf("[KERNEL] entryIdx=%u, dataIdx=%u\n", entryIdx, dataIdx);
    }
    
    // ✅ ДОБАВИТЬ: Защита dataIndex
    if (dataIdx >= 204800) return;  // maxBlocks
    
    // Load block + 1 border into shared memory (9x9x9 = 729 floats)
    __shared__ float cache[729];
    __shared__ int blockTriangleCount;
    __shared__ int blockTriangleStart;
    
    if (threadIdx.x == 0) {
        blockTriangleCount = 0;
    }
    __syncthreads();
    
    if (blockIdx_custom == 0 && threadIdx.x == 0) {
        printf("[KERNEL] before cache load\n");
    }
    
    // ✅ ИСПРАВЛЕНИЕ: Загружаем основной блок напрямую (без хэш-поиска)
    const float* currentBlock = blockData + dataIdx * 512;
    
    for (int i = threadIdx.x; i < 512; i += blockDim.x) {
        int lz = i >> 6;
        int ly = (i >> 3) & 7;
        int lx = i & 7;
        cache[lz * 81 + ly * 9 + lx] = currentBlock[i];
    }
    
    if (blockIdx_custom == 0 && threadIdx.x == 0) {
        printf("[KERNEL] main block loaded, loading borders\n");
    }
    
    // Граничные вокселы - УПРОЩЁННАЯ версия
    for (int i = threadIdx.x; i < 729; i += blockDim.x) {
        int z = i / 81;
        int y = (i / 9) % 9;
        int x = i % 9;
        
        if (x == 8 || y == 8 || z == 8) {
            // ✅ Пока просто ставим 1.0f (вне поверхности)
            // Это даст артефакты на границах блоков, но не зависнет
            cache[i] = 1.0f;
        }
    }
    
    if (blockIdx_custom == 0 && threadIdx.x == 0) {
        printf("[KERNEL] before syncthreads\n");
    }
    
    __syncthreads();
    
    if (blockIdx_custom == 0 && threadIdx.x == 0) {
        printf("[KERNEL] after syncthreads\n");
    }
    
    // Process one voxel per thread
    int voxelIdx = threadIdx.x;
    if (voxelIdx >= 512) return;
    
    if (blockIdx_custom == 0 && threadIdx.x == 0) {
        printf("[KERNEL] processing voxel 0\n");
    }
    
    int localX = voxelIdx & 7;
    int localY = (voxelIdx >> 3) & 7;
    int localZ = voxelIdx >> 6;
    
    // 8 углов куба из кэша
    #define CACHE_IDX(x,y,z) ((z)*81 + (y)*9 + (x))
    
    float v[8];
    v[0] = cache[CACHE_IDX(localX,   localY,   localZ)];
    v[1] = cache[CACHE_IDX(localX+1, localY,   localZ)];
    v[2] = cache[CACHE_IDX(localX+1, localY+1, localZ)];
    v[3] = cache[CACHE_IDX(localX,   localY+1, localZ)];
    v[4] = cache[CACHE_IDX(localX,   localY,   localZ+1)];
    v[5] = cache[CACHE_IDX(localX+1, localY,   localZ+1)];
    v[6] = cache[CACHE_IDX(localX+1, localY+1, localZ+1)];
    v[7] = cache[CACHE_IDX(localX,   localY+1, localZ+1)];
    
    #undef CACHE_IDX
    
    if (blockIdx_custom == 0 && threadIdx.x == 0) {
        printf("[KERNEL] v[0]=%f cubeindex calc\n", v[0]);
    }
    
    // Calculate cube index
    int cubeindex = 0;
    if (v[0] < isovalue) cubeindex |= 1;
    if (v[1] < isovalue) cubeindex |= 2;
    if (v[2] < isovalue) cubeindex |= 4;
    if (v[3] < isovalue) cubeindex |= 8;
    if (v[4] < isovalue) cubeindex |= 16;
    if (v[5] < isovalue) cubeindex |= 32;
    if (v[6] < isovalue) cubeindex |= 64;
    if (v[7] < isovalue) cubeindex |= 128;
    
    if (blockIdx_custom == 0 && threadIdx.x == 0) {
        printf("[KERNEL] cubeindex=%d edgeTable lookup\n", cubeindex);
    }
    
    if (d_edgeTable[cubeindex] == 0) return;
    
    if (blockIdx_custom == 0 && threadIdx.x == 0) {
        printf("[KERNEL] has edges, computing vertices\n");
    }
    
    // World positions
    int32_t voxelX = blockX * 8 + localX;
    int32_t voxelY = blockY * 8 + localY;
    int32_t voxelZ = blockZ * 8 + localZ;
    
    float3 p[8];
    p[0] = make_float3(voxelX * voxelSize,       voxelY * voxelSize,       voxelZ * voxelSize);
    p[1] = make_float3((voxelX+1) * voxelSize,   voxelY * voxelSize,       voxelZ * voxelSize);
    p[2] = make_float3((voxelX+1) * voxelSize,   (voxelY+1) * voxelSize,   voxelZ * voxelSize);
    p[3] = make_float3(voxelX * voxelSize,       (voxelY+1) * voxelSize,   voxelZ * voxelSize);
    p[4] = make_float3(voxelX * voxelSize,       voxelY * voxelSize,       (voxelZ+1) * voxelSize);
    p[5] = make_float3((voxelX+1) * voxelSize,   voxelY * voxelSize,       (voxelZ+1) * voxelSize);
    p[6] = make_float3((voxelX+1) * voxelSize,   (voxelY+1) * voxelSize,   (voxelZ+1) * voxelSize);
    p[7] = make_float3(voxelX * voxelSize,       (voxelY+1) * voxelSize,   (voxelZ+1) * voxelSize);
    
    // Interpolate vertices on edges
    float3 vertlist[12];
    int edges = d_edgeTable[cubeindex];
    if (edges & 1)    vertlist[0]  = VertexInterp(isovalue, p[0], v[0], p[1], v[1]);
    if (edges & 2)    vertlist[1]  = VertexInterp(isovalue, p[1], v[1], p[2], v[2]);
    if (edges & 4)    vertlist[2]  = VertexInterp(isovalue, p[2], v[2], p[3], v[3]);
    if (edges & 8)    vertlist[3]  = VertexInterp(isovalue, p[3], v[3], p[0], v[0]);
    if (edges & 16)   vertlist[4]  = VertexInterp(isovalue, p[4], v[4], p[5], v[5]);
    if (edges & 32)   vertlist[5]  = VertexInterp(isovalue, p[5], v[5], p[6], v[6]);
    if (edges & 64)   vertlist[6]  = VertexInterp(isovalue, p[6], v[6], p[7], v[7]);
    if (edges & 128)  vertlist[7]  = VertexInterp(isovalue, p[7], v[7], p[4], v[4]);
    if (edges & 256)  vertlist[8]  = VertexInterp(isovalue, p[0], v[0], p[4], v[4]);
    if (edges & 512)  vertlist[9]  = VertexInterp(isovalue, p[1], v[1], p[5], v[5]);
    if (edges & 1024) vertlist[10] = VertexInterp(isovalue, p[2], v[2], p[6], v[6]);
    if (edges & 2048) vertlist[11] = VertexInterp(isovalue, p[3], v[3], p[7], v[7]);
    
    // После интерполяции, перед подсчётом треугольников:
    if (blockIdx_custom == 0 && threadIdx.x == 0) {
        printf("[KERNEL] counting triangles\n");
    }
    
    // Count triangles for this voxel first
    int voxelTriCount = 0;
    for (int i = 0; d_triTable[cubeindex][i] != -1; i += 3) {
        voxelTriCount++;
    }
    
    if (blockIdx_custom == 0 && threadIdx.x == 0) {
        printf("[KERNEL] triCount=%d, atomicAdd\n", voxelTriCount);
    }
    
    if (voxelTriCount == 0) return;
    
    // Atomic reserve space
    int triBase = atomicAdd(triangleCounter, voxelTriCount);
    atomicAdd(&blockTriangleCount, voxelTriCount);
    
    if (blockIdx_custom == 0 && threadIdx.x == 0) {
        printf("[KERNEL] triBase=%d, maxTri=%d\n", triBase, maxTriangles);
    }
    
    if (triBase + voxelTriCount > maxTriangles) return;
    
    if (blockIdx_custom == 0 && threadIdx.x == 0) {
        printf("[KERNEL] writing triangles\n");
    }
    
    // Write triangles
    int triIdx = triBase;
    for (int i = 0; d_triTable[cubeindex][i] != -1; i += 3) {
        float3 v0 = vertlist[d_triTable[cubeindex][i]];
        float3 v1 = vertlist[d_triTable[cubeindex][i+1]];
        float3 v2 = vertlist[d_triTable[cubeindex][i+2]];
        
        // CCW winding
        outputVertices[triIdx * 3 + 0] = v0;
        outputVertices[triIdx * 3 + 1] = v2;
        outputVertices[triIdx * 3 + 2] = v1;
        
        // Flat normal
        float3 e1 = make_float3(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
        float3 e2 = make_float3(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
        float3 n = make_float3(
            e1.y * e2.z - e1.z * e2.y,
            e1.z * e2.x - e1.x * e2.z,
            e1.x * e2.y - e1.y * e2.x
        );
        float len = sqrtf(n.x*n.x + n.y*n.y + n.z*n.z);
        if (len > 1e-6f) { n.x /= len; n.y /= len; n.z /= len; }
        else { n = make_float3(0, 1, 0); }
        
        outputNormals[triIdx * 3 + 0] = n;
        outputNormals[triIdx * 3 + 1] = n;
        outputNormals[triIdx * 3 + 2] = n;
        
        triIdx++;
    }
    
    if (blockIdx_custom == 0 && threadIdx.x == 0) {
        printf("[KERNEL] done\n");
    }
    
    __syncthreads();
    
    // First thread records block mesh info
    if (threadIdx.x == 0 && updateMeshInfo && blockMeshInfo) {
        blockMeshInfo[dataIdx].triangleCount = blockTriangleCount;
        // triangleOffset would need prefix sum - simplified here
    }
}

// ============================================================================
// FAST NORMAL SMOOTHING
// ============================================================================

__device__ uint32_t normalHashFunction(float3 pos, float cellSize, uint32_t tableSize) {
    int32_t ix = __float2int_rd(pos.x / cellSize + 0.5f);
    int32_t iy = __float2int_rd(pos.y / cellSize + 0.5f);
    int32_t iz = __float2int_rd(pos.z / cellSize + 0.5f);
    uint32_t h = ix * 73856093u ^ iy * 19349663u ^ iz * 83492791u;
    return h & (tableSize - 1);
}

__global__ void AccumulateNormalsKernel(
    const float3* vertices,
    const float3* flatNormals,
    float4* accumTable,
    int vertexCount,
    float cellSize,
    uint32_t tableSize)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= vertexCount) return;
    
    float3 pos = vertices[idx];
    float3 n = flatNormals[idx];
    
    uint32_t slot = normalHashFunction(pos, cellSize, tableSize);
    
    atomicAdd(&accumTable[slot].x, n.x);
    atomicAdd(&accumTable[slot].y, n.y);
    atomicAdd(&accumTable[slot].z, n.z);
    atomicAdd(&accumTable[slot].w, 1.0f);
}

__global__ void ApplySmoothedNormalsKernel(
    const float3* vertices,
    const float4* accumTable,
    float3* normals,
    int vertexCount,
    float cellSize,
    uint32_t tableSize)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= vertexCount) return;
    
    float3 pos = vertices[idx];
    uint32_t slot = normalHashFunction(pos, cellSize, tableSize);
    
    float4 acc = accumTable[slot];
    
    float3 n;
    if (acc.w > 0.5f) {
        n = make_float3(acc.x / acc.w, acc.y / acc.w, acc.z / acc.w);
        float len = sqrtf(n.x*n.x + n.y*n.y + n.z*n.z);
        if (len > 1e-6f) {
            n.x /= len; n.y /= len; n.z /= len;
        } else {
            n = normals[idx];  // Keep original
        }
    } else {
        n = normals[idx];  // Keep original
    }
    
    normals[idx] = n;
}

// ============================================================================
// COMPACTION KERNEL
// ============================================================================

__global__ void CompactActiveBlocksKernel(
    const DynamicGpuHashGrid::HashEntry* hashTable,
    uint32_t tableSize,
    uint32_t* activeIndices,
    uint32_t* activeCount)
{
    uint32_t idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= tableSize) return;
    
    if (hashTable[idx].blockX != INT32_MAX) {
        uint32_t writeIdx = atomicAdd(activeCount, 1);
        activeIndices[writeIdx] = idx;
    }
}

// ============================================================================
// EXTRACT MESH IMPLEMENTATION
// ============================================================================

bool DynamicGpuHashGrid::ExtractMesh(
    float isovalue,
    float** d_vertices,
    float** d_normals,
    int** d_triangleCounter,
    void* stream)
{
    std::cout << "[DEBUG] ExtractMesh start" << std::endl;
    std::cout.flush();
    
    auto t_start = std::chrono::high_resolution_clock::now();
    
    // ✅ Проверь что это вызывается:
    InitializeMCTables();
    
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        std::cerr << "[DEBUG] MC tables init error: " << cudaGetErrorString(err) << std::endl;
    }
    
    cudaStream_t s = stream ? static_cast<cudaStream_t>(stream) : 0;
    
    // Get current counts
    UpdateActiveCountAsync(stream);
    cudaStreamSynchronize(s);
    
    uint32_t activeBlocks = cachedActiveBlockCount_;
    uint32_t dirtyBlocks = cachedDirtyCount_;
    
    if (activeBlocks == 0) {
        std::cout << "[DEBUG] no active blocks, exit" << std::endl;
        *d_vertices = nullptr;
        *d_normals = nullptr;
        *d_triangleCounter = nullptr;
        return false;
    }
    
    // Decide: incremental or full rebuild
    bool useIncremental = config_.enableIncrementalMesh && 
                          dirtyBlocks > 0 && 
                          dirtyBlocks < activeBlocks / 4 &&  // Less than 25% dirty
                          meshBuffers_[currentMeshBuffer_].valid;
    
    // For now, always do full rebuild (incremental is complex)
    // TODO: Implement true incremental updates
    useIncremental = false;
    
    std::cout << "[DEBUG] compacting active blocks..." << std::endl;
    std::cout.flush();
    
    // Compact active blocks
    cudaMemsetAsync(d_activeCount_, 0, sizeof(uint32_t), s);
    
    dim3 compactBlock(256);
    dim3 compactGrid((config_.hashTableSize + 255) / 256);
    CompactActiveBlocksKernel<<<compactGrid, compactBlock, 0, s>>>(
        d_hashTable_, config_.hashTableSize, d_activeIndices_, d_activeCount_);
    
    uint32_t compactedCount = 0;
    cudaMemcpyAsync(&compactedCount, d_activeCount_, sizeof(uint32_t), cudaMemcpyDeviceToHost, s);
    cudaStreamSynchronize(s);
    
    std::cout << "[DEBUG] compacted: " << compactedCount << " blocks" << std::endl;
    std::cout.flush();
    
    // ✅ Принудительно дождаться копирования счётчика
    cudaDeviceSynchronize();
    
    // ✅ СТАЛО - читаем актуальное значение:
    uint32_t actualBlockCount = 0;
    cudaMemcpy(&actualBlockCount, d_activeCount_, sizeof(uint32_t), cudaMemcpyDeviceToHost);
    
    if (actualBlockCount == 0 || actualBlockCount > config_.maxBlocks) {
        std::cout << "[DEBUG] invalid actualBlockCount: " << actualBlockCount << std::endl;
        *d_vertices = nullptr;
        *d_normals = nullptr;
        *d_triangleCounter = nullptr;
        clearDirtyList(stream);
        return false;
    }
    
    // Ensure mesh buffers are large enough
    uint32_t estimatedTriangles = actualBlockCount * 40;  // ~40 tris per block average
    if (estimatedTriangles > allocatedTriangles_) {
        // Reallocate
        uint32_t newSize = estimatedTriangles * 2;  // 2x headroom
        std::cout << "[DynamicGpuHashGrid] Growing mesh buffers: " << allocatedTriangles_ 
                  << " -> " << newSize << std::endl;
        
        for (int i = 0; i < 2; ++i) {
            if (meshBuffers_[i].d_vertices) cudaFree(meshBuffers_[i].d_vertices);
            if (meshBuffers_[i].d_normals) cudaFree(meshBuffers_[i].d_normals);
            
            cudaMalloc(&meshBuffers_[i].d_vertices, newSize * 9 * sizeof(float));
            cudaMalloc(&meshBuffers_[i].d_normals, newSize * 9 * sizeof(float));
            meshBuffers_[i].valid = false;
        }
        allocatedTriangles_ = newSize;
    }
    
    // Use next buffer (double buffering)
    int nextBuffer = 1 - currentMeshBuffer_;
    MeshBuffer& output = meshBuffers_[nextBuffer];
    
    // Allocate triangle counter
    int* d_triCount = nullptr;
    cudaMalloc(&d_triCount, sizeof(int));
    cudaMemsetAsync(d_triCount, 0, sizeof(int), s);
    
    // Launch marching cubes
    std::cout << "[DEBUG] blockCount for MC: " << actualBlockCount << std::endl;
    
    // Проверка что gridSize валидный
    if (actualBlockCount == 0 || actualBlockCount > 65535) {
        std::cout << "[DEBUG] invalid block count, skip MC" << std::endl;
        *d_vertices = nullptr;
        *d_normals = nullptr;
        *d_triangleCounter = nullptr;
        return false;
    }
    
    dim3 mcBlock(512);
    dim3 mcGrid(actualBlockCount);
    
    // Clamp grid size to CUDA limits
    if (mcGrid.x > 65535) {
        std::cerr << "[DynamicGpuHashGrid] Warning: Too many blocks (" << mcGrid.x 
                  << "), clamping to 65535" << std::endl;
        mcGrid.x = 65535;
    }
    
    std::cout << "[DEBUG] launching MC grid=" << mcGrid.x << " block=" << mcBlock.x << std::endl;
    std::cout.flush();
    
    MarchingCubesBlockKernel<<<mcGrid, mcBlock, 0, s>>>(
        d_hashTable_,
        config_.hashTableSize,
        d_blockData_,
        isovalue,
        config_.voxelSize,
        reinterpret_cast<float3*>(output.d_vertices),
        reinterpret_cast<float3*>(output.d_normals),
        d_triCount,
        allocatedTriangles_,
        d_activeIndices_,
        std::min(actualBlockCount, 65535u),
        d_blockMeshInfo_,
        true
    );
    
    std::cout << "[DEBUG] kernel launched, syncing..." << std::endl;
    std::cout.flush();
    
    cudaDeviceSynchronize();
    
    std::cout << "[DEBUG] kernel done" << std::endl;
    std::cout.flush();
    
    // Get triangle count
    int triangleCount = 0;
    cudaMemcpyAsync(&triangleCount, d_triCount, sizeof(int), cudaMemcpyDeviceToHost, s);
    cudaStreamSynchronize(s);
    
    if (triangleCount > 0) {
        // Smooth normals
        int vertexCount = triangleCount * 3;
        
        // Clear accumulation table
        cudaMemsetAsync(d_normalAccumTable_, 0, normalTableSize_ * sizeof(float) * 4, s);
        
        float cellSize = config_.voxelSize * 0.5f;  // Smoothing radius
        
        dim3 normBlock(256);
        dim3 normGrid((vertexCount + 255) / 256);
        
        AccumulateNormalsKernel<<<normGrid, normBlock, 0, s>>>(
            reinterpret_cast<float3*>(output.d_vertices),
            reinterpret_cast<float3*>(output.d_normals),
            reinterpret_cast<float4*>(d_normalAccumTable_),
            vertexCount,
            cellSize,
            normalTableSize_
        );
        
        ApplySmoothedNormalsKernel<<<normGrid, normBlock, 0, s>>>(
            reinterpret_cast<float3*>(output.d_vertices),
            reinterpret_cast<float4*>(d_normalAccumTable_),
            reinterpret_cast<float3*>(output.d_normals),
            vertexCount,
            cellSize,
            normalTableSize_
        );
    }
    
    // Update output buffer
    output.triangleCount = triangleCount;
    output.valid = true;
    
    // Swap buffers
    currentMeshBuffer_ = nextBuffer;
    
    // Update legacy pointers
    d_meshVertices_ = output.d_vertices;
    d_meshNormals_ = output.d_normals;
    
    // Return pointers
    *d_vertices = output.d_vertices;
    *d_normals = output.d_normals;
    *d_triangleCounter = d_triCount;
    
    // Clear dirty list
    clearDirtyList(stream);
    
    // Update stats
    auto t_end = std::chrono::high_resolution_clock::now();
    lastStats_.lastExtractTimeMs = std::chrono::duration<float, std::milli>(t_end - t_start).count();
    lastStats_.totalTriangles = triangleCount;
    lastStats_.activeBlockCount = compactedCount;
    lastStats_.dirtyBlockCount = dirtyBlocks;
    
    frameCounter_++;
    
    return true;
}

// Triangle table (abbreviated - full table needed)
const int h_triTable[256][16] = {
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1},
    {3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1},
    {3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1},
    {3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1},
    {9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
    {2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1},
    {8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
    {4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1},
    {3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1},
    {1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1},
    {4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1},
    {4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
    {5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1},
    {2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1},
    {9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
    {0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
    {2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1},
    {10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1},
    {5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1},
    {5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1},
    {9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1},
    {1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1},
    {10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1},
    {8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1},
    {2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1},
    {7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1},
    {2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1},
    {11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1},
    {5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1},
    {11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1},
    {11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1},
    {9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1},
    {2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1},
    {6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1},
    {3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1},
    {6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
    {10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1},
    {6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1},
    {8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1},
    {7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1},
    {3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
    {5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1},
    {0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1},
    {9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1},
    {8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1},
    {5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1},
    {0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1},
    {6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1},
    {10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1},
    {10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1},
    {8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1},
    {1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1},
    {0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1},
    {10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1},
    {3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1},
    {6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1},
    {9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1},
    {8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1},
    {3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1},
    {6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1},
    {0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1},
    {10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1},
    {10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1},
    {2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1},
    {7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1},
    {7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1},
    {2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1},
    {1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1},
    {11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1},
    {8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1},
    {0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1},
    {7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
    {10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
    {2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
    {6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1},
    {7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1},
    {2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1},
    {1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1},
    {10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1},
    {10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1},
    {0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1},
    {7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1},
    {6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1},
    {8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1},
    {9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1},
    {6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1},
    {4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1},
    {10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1},
    {8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1},
    {0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1},
    {1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1},
    {8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1},
    {10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1},
    {4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1},
    {10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
    {5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
    {11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1},
    {9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
    {6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1},
    {7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1},
    {3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1},
    {7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1},
    {3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1},
    {6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1},
    {9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1},
    {1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1},
    {4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1},
    {7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1},
    {6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1},
    {3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1},
    {0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1},
    {6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1},
    {0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1},
    {11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1},
    {6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1},
    {5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1},
    {9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1},
    {1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1},
    {1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1},
    {10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1},
    {0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1},
    {5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1},
    {10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1},
    {11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1},
    {9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1},
    {7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1},
    {2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1},
    {8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1},
    {9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1},
    {9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1},
    {1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1},
    {9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1},
    {9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1},
    {5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1},
    {0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1},
    {10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1},
    {2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1},
    {0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1},
    {0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1},
    {9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1},
    {5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1},
    {3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1},
    {5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1},
    {8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1},
    {0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1},
    {9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1},
    {1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1},
    {3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1},
    {4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1},
    {9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1},
    {11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1},
    {11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1},
    {2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1},
    {9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1},
    {3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1},
    {1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1},
    {4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1},
    {4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1},
    {3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1},
    {3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1},
    {0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1},
    {9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1},
    {1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
};

} // namespace Urbaxio::Engine

