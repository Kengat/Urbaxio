#pragma once

#include <vector>
#include <glm/glm.hpp>
#include <openvdb/openvdb.h>
#include <openvdb/tools/Dense.h>

namespace Urbaxio::Engine {

// Sparse 3D grid storing scalar SDF values using OpenVDB.
// This structure dramatically reduces memory usage by only storing values near the surface.
struct VoxelGrid {
    glm::uvec3 dimensions;     // Logical grid dimensions (bounding box)
    glm::vec3 origin;          // World-space origin of the grid
    float voxelSize;           // Size of each voxel in world units
    openvdb::FloatGrid::Ptr grid_;  // Sparse OpenVDB grid
    float backgroundValue_;    // Default value for empty voxels (positive = outside)

    // Constructor: creates an empty sparse grid
    VoxelGrid(const glm::uvec3& dims, const glm::vec3& worldOrigin, float size)
        : dimensions(dims), origin(worldOrigin), voxelSize(size), 
          backgroundValue_(size * 5.0f) // Default to "far outside"
    {
        // Initialize OpenVDB (must be called once in the application lifetime)
        static bool openvdb_initialized = false;
        if (!openvdb_initialized) {
            openvdb::initialize();
            openvdb_initialized = true;
        }
        
        // Create sparse grid with background value
        grid_ = openvdb::FloatGrid::create(backgroundValue_);
        grid_->setName("SDF_Grid");
        grid_->setGridClass(openvdb::GRID_LEVEL_SET);
        
        // Set voxel size and origin transform
        auto xform = openvdb::math::Transform::createLinearTransform(voxelSize);
        xform->postTranslate(openvdb::Vec3d(worldOrigin.x, worldOrigin.y, worldOrigin.z));
        grid_->setTransform(xform);
    }

    // Get value at voxel coordinates (read-only, efficient)
    float getValue(unsigned int x, unsigned int y, unsigned int z) const {
        if (!isValid(x, y, z)) return backgroundValue_;
        openvdb::Coord xyz(x, y, z);
        return grid_->tree().getValue(xyz);
    }

    // Set value at voxel coordinates (write, will activate sparse voxel if needed)
    void setValue(unsigned int x, unsigned int y, unsigned int z, float value) {
        if (!isValid(x, y, z)) return;
        openvdb::Coord xyz(x, y, z);
        grid_->tree().setValue(xyz, value);
    }

    // High-performance accessor for batch operations (use in tight loops)
    class Accessor {
    public:
        Accessor(VoxelGrid& grid) : acc_(grid.grid_->getAccessor()), grid_(grid) {}
        
        float getValue(unsigned int x, unsigned int y, unsigned int z) const {
            if (!grid_.isValid(x, y, z)) return grid_.backgroundValue_;
            return acc_.getValue(openvdb::Coord(x, y, z));
        }
        
        void setValue(unsigned int x, unsigned int y, unsigned int z, float value) {
            if (!grid_.isValid(x, y, z)) return;
            acc_.setValue(openvdb::Coord(x, y, z), value);
        }
        
    private:
        openvdb::FloatGrid::Accessor acc_;
        VoxelGrid& grid_;
    };

    class ConstAccessor {
    public:
        ConstAccessor(const VoxelGrid& grid) : acc_(grid.grid_->getConstAccessor()), grid_(grid) {}
        
        float getValue(unsigned int x, unsigned int y, unsigned int z) const {
            if (!grid_.isValid(x, y, z)) return grid_.backgroundValue_;
            return acc_.getValue(openvdb::Coord(x, y, z));
        }
        
    private:
        openvdb::FloatGrid::ConstAccessor acc_;
        const VoxelGrid& grid_;
    };

    // Bounds checking
    bool isValid(unsigned int x, unsigned int y, unsigned int z) const {
        return x < dimensions.x && y < dimensions.y && z < dimensions.z;
    }

    // Convert sparse OpenVDB grid to dense array (needed for Marching Cubes)
    std::vector<float> toDenseArray() const {
        std::vector<float> denseData(dimensions.x * dimensions.y * dimensions.z, backgroundValue_);
        
        // Use OpenVDB iterator to efficiently copy only active values
        auto accessor = grid_->getConstAccessor();
        for (unsigned int z = 0; z < dimensions.z; ++z) {
            for (unsigned int y = 0; y < dimensions.y; ++y) {
                for (unsigned int x = 0; x < dimensions.x; ++x) {
                    size_t index = z * dimensions.x * dimensions.y + y * dimensions.x + x;
                    denseData[index] = accessor.getValue(openvdb::Coord(x, y, z));
                }
            }
        }
        return denseData;
    }

    // Create sparse grid from dense array (needed for Undo/Redo)
    void fromDenseArray(const std::vector<float>& denseData) {
        if (denseData.size() != dimensions.x * dimensions.y * dimensions.z) {
            return; // Size mismatch
        }
        
        // Clear existing data
        grid_->clear();
        
        // Use accessor for efficient batch writes
        auto accessor = grid_->getAccessor();
        for (unsigned int z = 0; z < dimensions.z; ++z) {
            for (unsigned int y = 0; y < dimensions.y; ++y) {
                for (unsigned int x = 0; x < dimensions.x; ++x) {
                    size_t index = z * dimensions.x * dimensions.y + y * dimensions.x + x;
                    float value = denseData[index];
                    
                    // Only store values that differ from background (sparsity optimization)
                    if (std::abs(value - backgroundValue_) > 1e-6f) {
                        accessor.setValue(openvdb::Coord(x, y, z), value);
                    }
                }
            }
        }
    }

    // Get memory usage statistics
    size_t getActiveVoxelCount() const {
        return grid_->activeVoxelCount();
    }

    size_t getMemoryUsage() const {
        return grid_->memUsage();
    }
};

} // namespace Urbaxio::Engine

