#pragma once

#include <vector>
#include <glm/glm.hpp>

namespace Urbaxio::Engine {

// A simple dense 3D grid storing scalar values (SDF).
struct VoxelGrid {
    glm::uvec3 dimensions;
    glm::vec3 origin;
    float voxelSize;
    std::vector<float> sdfData;

    VoxelGrid(const glm::uvec3& dims, const glm::vec3& worldOrigin, float size)
        : dimensions(dims), origin(worldOrigin), voxelSize(size) {
        sdfData.resize(dimensions.x * dimensions.y * dimensions.z, voxelSize * 5.0f); // Default to empty
    }

    // Accessor to treat 1D vector as a 3D grid.
    float& at(unsigned int x, unsigned int y, unsigned int z) {
        return sdfData[z * dimensions.x * dimensions.y + y * dimensions.x + x];
    }

    const float& at(unsigned int x, unsigned int y, unsigned int z) const {
        return sdfData[z * dimensions.x * dimensions.y + y * dimensions.x + x];
    }

    bool isValid(unsigned int x, unsigned int y, unsigned int z) const {
        return x < dimensions.x && y < dimensions.y && z < dimensions.z;
    }
};

} // namespace Urbaxio::Engine

