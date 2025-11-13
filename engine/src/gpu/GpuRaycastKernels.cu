// GPU Raycast Kernels - Direct ray tracing against NanoVDB SDF

// Eliminates CPU BVH and "Stale Mesh Bug"

#include "engine/gpu/GpuRaycastKernels.h"

#include <cuda_runtime.h>

#ifdef __CUDACC__

#pragma nv_diag_suppress 20054

#pragma nv_diag_suppress 20012

#endif

#define OPENVDB_USE_DELAYED_LOADING 1

#define HALF_ENABLE_F16C_INTRINSICS 0

#include <nanovdb/NanoVDB.h>

#include <iostream>

namespace Urbaxio::Engine {

// ============================================================================

// Sphere Tracing Kernel for SDF Grids

// ============================================================================

__global__ void SphereTracingKernel(

    const nanovdb::FloatGrid* grid,

    float3 rayOrigin,

    float3 rayDirection,

    float maxDistance,

    float isoValue,

    bool* hit_out,

    float3* hitPoint_out,

    float3* hitNormal_out,

    float* hitDistance_out

)

{

    // Single thread kernel (called with <<<1, 1>>>)

    if (threadIdx.x != 0 || blockIdx.x != 0) return;

    auto acc = grid->tree().getAccessor();

    

    float t = 0.0f;

    const float epsilon = 0.001f;  // Stop threshold

    const int maxSteps = 256;

    

    bool foundHit = false;

    float3 currentPos;

    float finalT = 0.0f;

    

    // Sphere tracing loop

    for (int step = 0; step < maxSteps && t < maxDistance; ++step) {

        currentPos = make_float3(

            rayOrigin.x + rayDirection.x * t,

            rayOrigin.y + rayDirection.y * t,

            rayOrigin.z + rayDirection.z * t

        );

        

        // Convert world position to index space

        nanovdb::Vec3f worldPos(currentPos.x, currentPos.y, currentPos.z);

        nanovdb::Vec3f indexPos = grid->worldToIndexF(worldPos);

        

        // Sample SDF at current position

        float sdfValue = acc.getValue(nanovdb::Coord(

            __float2int_rn(indexPos[0]),

            __float2int_rn(indexPos[1]),

            __float2int_rn(indexPos[2])

        ));

        

        // Check if we crossed the isosurface

        float dist = sdfValue - isoValue;

        

        if (fabsf(dist) < epsilon) {

            // Hit found!

            foundHit = true;

            finalT = t;

            break;

        }

        

        if (dist < -epsilon) {

            // Overshot - binary search refinement

            float tMin = t - fabsf(sdfValue);

            float tMax = t;

            

            for (int refine = 0; refine < 4; ++refine) {

                float tMid = (tMin + tMax) * 0.5f;

                float3 midPos = make_float3(

                    rayOrigin.x + rayDirection.x * tMid,

                    rayOrigin.y + rayDirection.y * tMid,

                    rayOrigin.z + rayDirection.z * tMid

                );

                

                nanovdb::Vec3f midWorldPos(midPos.x, midPos.y, midPos.z);

                nanovdb::Vec3f midIndexPos = grid->worldToIndexF(midWorldPos);

                

                float midSDF = acc.getValue(nanovdb::Coord(

                    __float2int_rn(midIndexPos[0]),

                    __float2int_rn(midIndexPos[1]),

                    __float2int_rn(midIndexPos[2])

                ));

                

                if (midSDF - isoValue > 0) {

                    tMin = tMid;

                } else {

                    tMax = tMid;

                }

            }

            

            foundHit = true;

            finalT = (tMin + tMax) * 0.5f;

            currentPos = make_float3(

                rayOrigin.x + rayDirection.x * finalT,

                rayOrigin.y + rayDirection.y * finalT,

                rayOrigin.z + rayDirection.z * finalT

            );

            break;

        }

        

        // Step forward by SDF distance (sphere tracing)

        t += fmaxf(fabsf(dist), epsilon);

    }

    

    // Output results

    *hit_out = foundHit;

    *hitDistance_out = finalT;

    

    if (foundHit) {

        *hitPoint_out = currentPos;

        

        // Calculate normal using central differences

        nanovdb::Vec3f worldPos(currentPos.x, currentPos.y, currentPos.z);

        nanovdb::Vec3f indexPos = grid->worldToIndexF(worldPos);

        nanovdb::Coord center(

            __float2int_rn(indexPos[0]),

            __float2int_rn(indexPos[1]),

            __float2int_rn(indexPos[2])

        );

        

        float h = 0.5f;  // Step size for gradient

        float dx = acc.getValue(nanovdb::Coord(center[0]+1, center[1], center[2])) -

                   acc.getValue(nanovdb::Coord(center[0]-1, center[1], center[2]));

        float dy = acc.getValue(nanovdb::Coord(center[0], center[1]+1, center[2])) -

                   acc.getValue(nanovdb::Coord(center[0], center[1]-1, center[2]));

        float dz = acc.getValue(nanovdb::Coord(center[0], center[1], center[2]+1)) -

                   acc.getValue(nanovdb::Coord(center[0], center[1], center[2]-1));

        

        float len = sqrtf(dx*dx + dy*dy + dz*dz);

        if (len > 0.0001f) {

            dx /= len; dy /= len; dz /= len;

        }

        

        *hitNormal_out = make_float3(dx, dy, dz);

    } else {

        *hitPoint_out = make_float3(0.0f, 0.0f, 0.0f);

        *hitNormal_out = make_float3(0.0f, 1.0f, 0.0f);

    }

}

// ============================================================================

// Host Interface

// ============================================================================

GpuRaycastKernels::RaycastResult GpuRaycastKernels::RaycastNanoVDB(

    void* deviceGridPtr,

    const glm::vec3& rayOrigin,

    const glm::vec3& rayDirection,

    float maxDistance,

    float isoValue)

{

    RaycastResult result;

    result.hit = false;

    result.hitDistance = maxDistance;

    

    if (!deviceGridPtr) {

        std::cerr << "[GpuRaycastKernels] Invalid grid pointer!" << std::endl;

        return result;

    }

    

    // Allocate device output buffers

    bool* d_hit = nullptr;

    float3* d_hitPoint = nullptr;

    float3* d_hitNormal = nullptr;

    float* d_hitDistance = nullptr;

    

    cudaMalloc(&d_hit, sizeof(bool));

    cudaMalloc(&d_hitPoint, sizeof(float3));

    cudaMalloc(&d_hitNormal, sizeof(float3));

    cudaMalloc(&d_hitDistance, sizeof(float));

    

    // Launch kernel (single thread)

    float3 origin = make_float3(rayOrigin.x, rayOrigin.y, rayOrigin.z);

    float3 direction = make_float3(rayDirection.x, rayDirection.y, rayDirection.z);

    

    SphereTracingKernel<<<1, 1>>>(

        reinterpret_cast<nanovdb::FloatGrid*>(deviceGridPtr),

        origin,

        direction,

        maxDistance,

        isoValue,

        d_hit,

        d_hitPoint,

        d_hitNormal,

        d_hitDistance

    );

    

    cudaError_t err = cudaGetLastError();

    if (err != cudaSuccess) {

        std::cerr << "[GpuRaycastKernels] Kernel error: " 

                  << cudaGetErrorString(err) << std::endl;

        cudaFree(d_hit);

        cudaFree(d_hitPoint);

        cudaFree(d_hitNormal);

        cudaFree(d_hitDistance);

        return result;

    }

    

    // Copy results back to host

    bool hit;

    float3 hitPoint;

    float3 hitNormal;

    float hitDistance;

    

    cudaMemcpy(&hit, d_hit, sizeof(bool), cudaMemcpyDeviceToHost);

    cudaMemcpy(&hitPoint, d_hitPoint, sizeof(float3), cudaMemcpyDeviceToHost);

    cudaMemcpy(&hitNormal, d_hitNormal, sizeof(float3), cudaMemcpyDeviceToHost);

    cudaMemcpy(&hitDistance, d_hitDistance, sizeof(float), cudaMemcpyDeviceToHost);

    

    // Cleanup

    cudaFree(d_hit);

    cudaFree(d_hitPoint);

    cudaFree(d_hitNormal);

    cudaFree(d_hitDistance);

    

    // Pack results

    result.hit = hit;

    result.hitDistance = hitDistance;

    

    if (hit) {

        result.hitPoint = glm::vec3(hitPoint.x, hitPoint.y, hitPoint.z);

        result.hitNormal = glm::vec3(hitNormal.x, hitNormal.y, hitNormal.z);

    }

    

    return result;

}

bool GpuRaycastKernels::IsAvailable() {

    int deviceCount = 0;

    cudaError_t error = cudaGetDeviceCount(&deviceCount);

    return (error == cudaSuccess && deviceCount > 0);

}

} // namespace Urbaxio::Engine

