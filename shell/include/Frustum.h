#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <limits>

namespace Urbaxio {

struct Frustum {
    glm::vec4 planes[6];

    void ExtractPlanes(const glm::mat4& vpMatrix) {
        // The Gribb/Hartmann method for plane extraction works on the rows of the
        // combined view-projection matrix. Since GLM matrices are column-major,
        // we must transpose the matrix first to access rows as if they were columns.
        const glm::mat4 matrix = glm::transpose(vpMatrix);

        planes[0] = matrix[3] + matrix[0]; // Left
        planes[1] = matrix[3] - matrix[0]; // Right
        planes[2] = matrix[3] + matrix[1]; // Bottom
        planes[3] = matrix[3] - matrix[1]; // Top
        planes[4] = matrix[3] + matrix[2]; // Near
        planes[5] = matrix[3] - matrix[2]; // Far

        // Normalize each plane for correct distance calculations
        for (int i = 0; i < 6; ++i) {
            float length = glm::length(glm::vec3(planes[i]));
            if (length > 1e-6) {
                planes[i] /= length;
            }
        }
    }

    // Test a world-space AABB against the frustum
    bool isAABBVisible_WorldSpace(const glm::vec3& aabbMin, const glm::vec3& aabbMax) const {
        for (int i = 0; i < 6; ++i) {
            const glm::vec4& plane = planes[i];

            // Find the p-vertex (positive vertex) of the AABB along the plane's normal.
            // This is the corner of the box that is furthest in the direction of the normal.
            glm::vec3 pVertex = aabbMin;
            if (plane.x > 0) pVertex.x = aabbMax.x;
            if (plane.y > 0) pVertex.y = aabbMax.y;
            if (plane.z > 0) pVertex.z = aabbMax.z;

            // If the p-vertex is on the negative side of the plane, the AABB is completely outside.
            if (glm::dot(glm::vec3(plane), pVertex) + plane.w < 0) {
                return false;
            }
        }
        return true;
    }

    // Test a local-space AABB by transforming it into a world-space AABB first
    bool IsAABBVisible(const glm::vec3& localAabbMin, const glm::vec3& localAabbMax, const glm::mat4& model) const {
        // If model matrix is identity, use the faster direct test
        if (model == glm::mat4(1.0f)) {
            return isAABBVisible_WorldSpace(localAabbMin, localAabbMax);
        }

        // Get the 8 corners of the local AABB
        glm::vec3 corners[8] = {
            {localAabbMin.x, localAabbMin.y, localAabbMin.z},
            {localAabbMax.x, localAabbMin.y, localAabbMin.z},
            {localAabbMax.x, localAabbMax.y, localAabbMin.z},
            {localAabbMin.x, localAabbMax.y, localAabbMin.z},
            {localAabbMin.x, localAabbMin.y, localAabbMax.z},
            {localAabbMax.x, localAabbMin.y, localAabbMax.z},
            {localAabbMax.x, localAabbMax.y, localAabbMax.z},
            {localAabbMin.x, localAabbMax.y, localAabbMax.z}
        };

        // Transform corners to world space and find the new AABB that encloses the transformed box
        glm::vec3 worldAabbMin(std::numeric_limits<float>::max());
        glm::vec3 worldAabbMax(std::numeric_limits<float>::lowest());

        for (int i = 0; i < 8; ++i) {
            glm::vec3 transformedCorner = glm::vec3(model * glm::vec4(corners[i], 1.0f));
            worldAabbMin = glm::min(worldAabbMin, transformedCorner);
            worldAabbMax = glm::max(worldAabbMax, transformedCorner);
        }
        
        // Test the new world-space AABB
        return isAABBVisible_WorldSpace(worldAabbMin, worldAabbMax);
    }
};

} // namespace Urbaxio

