#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <cstdint>
#include <glm/mat4x4.hpp>

namespace Urbaxio { class Camera; namespace Engine { class Scene; class SceneObject; } } // Added SceneObject

namespace Urbaxio {

    enum class SnapType {
        NONE,
        ENDPOINT,
        MIDPOINT,
        ON_EDGE,
        ON_FACE, // <<< NEW
        INTERSECTION,
        CENTER,
        ORIGIN,
        GRID,
        AXIS_X,
        AXIS_Y,
        AXIS_Z,
        PARALLEL,
        PERPENDICULAR
    };

    struct SnapResult {
        bool snapped = false;
        glm::vec3 worldPoint = glm::vec3(0.0f);
        SnapType type = SnapType::NONE;
    };

    class SnappingSystem {
    public:
        SnappingSystem();

        SnapResult FindSnapPoint(
            int mouseX, int mouseY, int screenWidth, int screenHeight,
            const Camera& camera, const Engine::Scene& scene,
            float snapThresholdPixels = 10.0f
        );

        // --- Static Helper Functions ---
        static bool WorldToScreen(
            const glm::vec3& worldPos,
            const glm::mat4& viewMatrix,
            const glm::mat4& projectionMatrix,
            int screenWidth, int screenHeight,
            glm::vec2& outScreenPos
        );

        static bool RaycastToZPlane(
            int mouseX, int mouseY,
            int screenWidth, int screenHeight,
            const Camera& camera,
            glm::vec3& outIntersectionPoint
        );

        // <<< NEW: Moved RayTriangleIntersect here >>>
        static bool RayTriangleIntersect(
            const glm::vec3& rayOrigin, const glm::vec3& rayDirection,
            const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2,
            float& t_intersection // Output: distance along ray to intersection point
        );
    };

} // namespace Urbaxio