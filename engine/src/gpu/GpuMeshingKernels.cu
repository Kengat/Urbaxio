#include "engine/gpu/GpuMeshingKernels.h"
#include <nanovdb/NanoVDB.h>
#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <iostream>
#include <vector>

namespace Urbaxio::Engine {

// Marching Cubes edge table (256 entries)
// Indicates which edges are intersected for each cube configuration
__constant__ int d_edgeTable[256];

// Marching Cubes triangle table (256 x 16 entries)
// Defines triangles for each cube configuration
__constant__ int d_triTable[256][16];

// Host-side lookup tables (initialized once)
static bool s_tablesInitialized = false;

// Simplified edge table (partial - full table would be 256 entries)
static const int h_edgeTable[256] = {
    0x0  , 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c,
    0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00,
    // ... (full table omitted for brevity - would include all 256 entries)
    // See Paul Bourke's Marching Cubes implementation for complete table
};

// Triangle table structure (partial)
static const int h_triTable[256][16] = {
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    {1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
    // ... (full table omitted for brevity)
};

// CUDA kernel for classifying voxels
__global__ void ClassifyVoxelsKernel(
    const nanovdb::FloatGrid* grid,
    float isoValue,
    int3 gridDim,
    int* voxelVerts,  // Output: number of vertices per voxel
    int* voxelOccupied // Output: 1 if voxel generates triangles, 0 otherwise
)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int z = blockIdx.z * blockDim.z + threadIdx.z;

    if (x >= gridDim.x - 1 || y >= gridDim.y - 1 || z >= gridDim.z - 1) return;

    int voxelIdx = z * gridDim.x * gridDim.y + y * gridDim.x + x;

    // Sample 8 corners of the voxel
    auto acc = grid->getAccessor();
    float v[8];
    v[0] = acc.getValue(nanovdb::Coord(x,   y,   z));
    v[1] = acc.getValue(nanovdb::Coord(x+1, y,   z));
    v[2] = acc.getValue(nanovdb::Coord(x+1, y,   z+1));
    v[3] = acc.getValue(nanovdb::Coord(x,   y,   z+1));
    v[4] = acc.getValue(nanovdb::Coord(x,   y+1, z));
    v[5] = acc.getValue(nanovdb::Coord(x+1, y+1, z));
    v[6] = acc.getValue(nanovdb::Coord(x+1, y+1, z+1));
    v[7] = acc.getValue(nanovdb::Coord(x,   y+1, z+1));

    // Calculate cube index
    int cubeIndex = 0;
    if (v[0] < isoValue) cubeIndex |= 1;
    if (v[1] < isoValue) cubeIndex |= 2;
    if (v[2] < isoValue) cubeIndex |= 4;
    if (v[3] < isoValue) cubeIndex |= 8;
    if (v[4] < isoValue) cubeIndex |= 16;
    if (v[5] < isoValue) cubeIndex |= 32;
    if (v[6] < isoValue) cubeIndex |= 64;
    if (v[7] < isoValue) cubeIndex |= 128;

    // Count vertices for this voxel
    int numVerts = 0;
    if (cubeIndex != 0 && cubeIndex != 255) {
        // Count number of vertices (-1 terminated list)
        for (int i = 0; d_triTable[cubeIndex][i] != -1; i++) {
            numVerts++;
        }
    }

    voxelVerts[voxelIdx] = numVerts;
    voxelOccupied[voxelIdx] = (numVerts > 0) ? 1 : 0;
}

// CUDA kernel for generating triangles
__global__ void GenerateTrianglesKernel(
    const nanovdb::FloatGrid* grid,
    float isoValue,
    int3 gridDim,
    float voxelSize,
    const int* voxelVerts,
    const int* voxelVertsScan, // Prefix sum of voxelVerts
    float3* outVertices
)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int z = blockIdx.z * blockDim.z + threadIdx.z;

    if (x >= gridDim.x - 1 || y >= gridDim.y - 1 || z >= gridDim.z - 1) return;

    int voxelIdx = z * gridDim.x * gridDim.y + y * gridDim.x + x;
    
    if (voxelVerts[voxelIdx] == 0) return; // No triangles for this voxel

    // Sample 8 corners of the voxel
    auto acc = grid->getAccessor();
    float v[8];
    v[0] = acc.getValue(nanovdb::Coord(x,   y,   z));
    v[1] = acc.getValue(nanovdb::Coord(x+1, y,   z));
    v[2] = acc.getValue(nanovdb::Coord(x+1, y,   z+1));
    v[3] = acc.getValue(nanovdb::Coord(x,   y,   z+1));
    v[4] = acc.getValue(nanovdb::Coord(x,   y+1, z));
    v[5] = acc.getValue(nanovdb::Coord(x+1, y+1, z));
    v[6] = acc.getValue(nanovdb::Coord(x+1, y+1, z+1));
    v[7] = acc.getValue(nanovdb::Coord(x,   y+1, z+1));

    // Calculate cube index
    int cubeIndex = 0;
    if (v[0] < isoValue) cubeIndex |= 1;
    if (v[1] < isoValue) cubeIndex |= 2;
    if (v[2] < isoValue) cubeIndex |= 4;
    if (v[3] < isoValue) cubeIndex |= 8;
    if (v[4] < isoValue) cubeIndex |= 16;
    if (v[5] < isoValue) cubeIndex |= 32;
    if (v[6] < isoValue) cubeIndex |= 64;
    if (v[7] < isoValue) cubeIndex |= 128;

    // Corner positions in world space
    float3 p[8];
    p[0] = make_float3(x * voxelSize,     y * voxelSize,     z * voxelSize);
    p[1] = make_float3((x+1) * voxelSize, y * voxelSize,     z * voxelSize);
    p[2] = make_float3((x+1) * voxelSize, y * voxelSize,     (z+1) * voxelSize);
    p[3] = make_float3(x * voxelSize,     y * voxelSize,     (z+1) * voxelSize);
    p[4] = make_float3(x * voxelSize,     (y+1) * voxelSize, z * voxelSize);
    p[5] = make_float3((x+1) * voxelSize, (y+1) * voxelSize, z * voxelSize);
    p[6] = make_float3((x+1) * voxelSize, (y+1) * voxelSize, (z+1) * voxelSize);
    p[7] = make_float3(x * voxelSize,     (y+1) * voxelSize, (z+1) * voxelSize);

    // Get output vertex offset for this voxel
    int vertexOffset = voxelVertsScan[voxelIdx];

    // Generate vertices for each triangle
    // (Simplified - full implementation would interpolate along edges)
    int vertIdx = 0;
    for (int i = 0; d_triTable[cubeIndex][i] != -1; i++) {
        int edge = d_triTable[cubeIndex][i];
        
        // Simplified: just output corner positions
        // Production code would interpolate along the edge based on SDF values
        float3 vertex = p[edge % 8]; // Placeholder
        
        outVertices[vertexOffset + vertIdx] = vertex;
        vertIdx++;
    }
}

void GpuMeshingKernels::InitializeLookupTables() {
    if (s_tablesInitialized) return;

    // Copy lookup tables to GPU constant memory
    cudaMemcpyToSymbol(d_edgeTable, h_edgeTable, sizeof(h_edgeTable));
    cudaMemcpyToSymbol(d_triTable, h_triTable, sizeof(h_triTable));

    s_tablesInitialized = true;
    std::cout << "[GpuMeshingKernels] Lookup tables initialized" << std::endl;
}

bool GpuMeshingKernels::MarchingCubes(
    void* deviceGridPtr,
    float isoValue,
    float voxelSize,
    CadKernel::MeshBuffers& outMesh)
{
    if (!deviceGridPtr) {
        std::cerr << "[GpuMeshingKernels] Invalid device grid pointer!" << std::endl;
        return false;
    }

    InitializeLookupTables();

    auto* grid = reinterpret_cast<nanovdb::FloatGrid*>(deviceGridPtr);

    // For this simplified implementation, assume a fixed grid size
    // In production, get actual bounds from NanoVDB grid metadata
    int3 gridDim = make_int3(128, 128, 128);
    int totalVoxels = gridDim.x * gridDim.y * gridDim.z;

    // Allocate device memory for classification
    thrust::device_vector<int> d_voxelVerts(totalVoxels);
    thrust::device_vector<int> d_voxelOccupied(totalVoxels);

    // Step 1: Classify voxels
    dim3 blockSize(8, 8, 8);
    dim3 gridSize(
        (gridDim.x + blockSize.x - 1) / blockSize.x,
        (gridDim.y + blockSize.y - 1) / blockSize.y,
        (gridDim.z + blockSize.z - 1) / blockSize.z
    );

    ClassifyVoxelsKernel<<<gridSize, blockSize>>>(
        grid,
        isoValue,
        gridDim,
        thrust::raw_pointer_cast(d_voxelVerts.data()),
        thrust::raw_pointer_cast(d_voxelOccupied.data())
    );

    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        std::cerr << "[GpuMeshingKernels] Classification kernel error: " 
                  << cudaGetErrorString(err) << std::endl;
        return false;
    }

    cudaDeviceSynchronize();

    // Step 2: Prefix sum to get vertex offsets
    thrust::device_vector<int> d_voxelVertsScan(totalVoxels);
    thrust::exclusive_scan(d_voxelVerts.begin(), d_voxelVerts.end(), 
                          d_voxelVertsScan.begin());

    // Get total vertex count
    int totalVertices = 0;
    cudaMemcpy(&totalVertices, 
               thrust::raw_pointer_cast(d_voxelVertsScan.data()) + totalVoxels - 1,
               sizeof(int), 
               cudaMemcpyDeviceToHost);

    if (totalVertices == 0) {
        std::cout << "[GpuMeshingKernels] No triangles generated (empty grid)" << std::endl;
        return true; // Not an error, just empty mesh
    }

    // Step 3: Allocate output buffers
    thrust::device_vector<float3> d_vertices(totalVertices);

    // Step 4: Generate triangles
    GenerateTrianglesKernel<<<gridSize, blockSize>>>(
        grid,
        isoValue,
        gridDim,
        voxelSize,
        thrust::raw_pointer_cast(d_voxelVerts.data()),
        thrust::raw_pointer_cast(d_voxelVertsScan.data()),
        thrust::raw_pointer_cast(d_vertices.data())
    );

    err = cudaGetLastError();
    if (err != cudaSuccess) {
        std::cerr << "[GpuMeshingKernels] Triangle generation kernel error: " 
                  << cudaGetErrorString(err) << std::endl;
        return false;
    }

    cudaDeviceSynchronize();

    // Step 5: Copy results back to host
    thrust::host_vector<float3> h_vertices = d_vertices;

    outMesh.vertices.resize(totalVertices * 3);
    for (int i = 0; i < totalVertices; ++i) {
        outMesh.vertices[i * 3 + 0] = h_vertices[i].x;
        outMesh.vertices[i * 3 + 1] = h_vertices[i].y;
        outMesh.vertices[i * 3 + 2] = h_vertices[i].z;
    }

    // Generate simple indices (each 3 vertices form a triangle)
    outMesh.indices.resize(totalVertices);
    for (int i = 0; i < totalVertices; ++i) {
        outMesh.indices[i] = i;
    }

    // Generate placeholder normals and UVs
    outMesh.normals.resize(totalVertices * 3, 0.0f);
    outMesh.uvs.resize(totalVertices * 2, 0.0f);

    std::cout << "[GpuMeshingKernels] Generated mesh with " << totalVertices 
              << " vertices and " << (totalVertices / 3) << " triangles" << std::endl;

    return true;
}

bool GpuMeshingKernels::IsAvailable() {
    int deviceCount = 0;
    cudaError_t error = cudaGetDeviceCount(&deviceCount);
    return (error == cudaSuccess && deviceCount > 0);
}

size_t GpuMeshingKernels::EstimateTriangleCount(void* deviceGridPtr, float isoValue) {
    // Simplified estimation - return a conservative estimate
    // In production, run classification kernel to get exact count
    return 100000; // Placeholder
}

} // namespace Urbaxio::Engine

