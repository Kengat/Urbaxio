#pragma once

#include <glm/glm.hpp>
#include <cstddef>

namespace Urbaxio::Engine {

/**
 * @brief GPU kernels for fast AABB computation
 */
class GpuAabbKernels {
public:
    /**
     * @brief Compute AABB from device vertex buffer
     * @param d_vertices Device pointer to vertices (x,y,z,x,y,z,...)
     * @param vertexCount Number of vertices
     * @param outMin Output minimum corner (host memory)
     * @param outMax Output maximum corner (host memory)
     * @param stream CUDA stream for async execution (optional)
     * @return True if successful
     */
    static bool ComputeAabb(
        const float* d_vertices,
        size_t vertexCount,
        glm::vec3& outMin,
        glm::vec3& outMax,
        void* stream = nullptr
    );
};

} // namespace Urbaxio::Engine


