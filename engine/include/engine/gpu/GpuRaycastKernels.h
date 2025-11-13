#pragma once

#include <glm/glm.hpp>

namespace Urbaxio::Engine {

class GpuRaycastKernels {

public:

    struct RaycastResult {

        bool hit;

        glm::vec3 hitPoint;

        glm::vec3 hitNormal;

        float hitDistance;

    };

    // Raycast against NanoVDB grid on GPU

    // Uses sphere-tracing for SDF grids (fast and accurate)

    static RaycastResult RaycastNanoVDB(

        void* deviceGridPtr,

        const glm::vec3& rayOrigin,

        const glm::vec3& rayDirection,

        float maxDistance = 1000.0f,

        float isoValue = 0.0f

    );

    static bool IsAvailable();

};

} // namespace Urbaxio::Engine

