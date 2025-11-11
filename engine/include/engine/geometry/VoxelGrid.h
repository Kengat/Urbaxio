#pragma once

#include <vector>
#include <glm/glm.hpp>
#include <openvdb/openvdb.h>
#include <openvdb/tools/Dense.h>

namespace Urbaxio::Engine {

// Sparse 3D grid storing scalar SDF values using OpenVDB.
// This structure dramatically reduces memory usage by only storing values near the surface.
// NEW: Supports unbounded grids with dynamic bounding box expansion.
struct VoxelGrid {
    glm::uvec3 dimensions;     // Initial/suggested dimensions (can grow dynamically)
    glm::vec3 origin;          // World-space origin of the grid
    float voxelSize;           // Size of each voxel in world units
    openvdb::FloatGrid::Ptr grid_;  // Sparse OpenVDB grid (unbounded)
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
    // NEW: No bounds checking - OpenVDB handles infinite grids naturally
    float getValue(int x, int y, int z) const {
        openvdb::Coord xyz(x, y, z);
        return grid_->tree().getValue(xyz);
    }

    // Set value at voxel coordinates (write, will activate sparse voxel if needed)
    // NEW: No bounds checking - grid expands automatically
    void setValue(int x, int y, int z, float value) {
        openvdb::Coord xyz(x, y, z);
        grid_->tree().setValue(xyz, value);
    }

    // High-performance accessor for batch operations (use in tight loops)
    class Accessor {
    public:
        Accessor(VoxelGrid& grid) : acc_(grid.grid_->getAccessor()), grid_(grid) {}
        
        float getValue(int x, int y, int z) const {
            return acc_.getValue(openvdb::Coord(x, y, z));
        }
        
        void setValue(int x, int y, int z, float value) {
            acc_.setValue(openvdb::Coord(x, y, z), value);
        }
        
    private:
        openvdb::FloatGrid::Accessor acc_;
        VoxelGrid& grid_;
    };

    class ConstAccessor {
    public:
        ConstAccessor(const VoxelGrid& grid) : acc_(grid.grid_->getConstAccessor()), grid_(grid) {}
        
        float getValue(int x, int y, int z) const {
            return acc_.getValue(openvdb::Coord(x, y, z));
        }
        
    private:
        openvdb::FloatGrid::ConstAccessor acc_;
        const VoxelGrid& grid_;
    };

    // DEPRECATED: Bounds checking no longer needed - kept for backward compatibility
    bool isValid(int x, int y, int z) const {
        return true; // Always valid - OpenVDB handles unbounded grids
    }

    // NEW: Get the actual bounding box of active voxels
    openvdb::CoordBBox getActiveBounds() const {
        return grid_->evalActiveVoxelBoundingBox();
    }

    // NEW: Update logical dimensions based on actual data
    void updateDimensions() {
        openvdb::CoordBBox bbox = getActiveBounds();
        if (!bbox.empty()) {
            // Expand dimensions to fit all active voxels (with some padding)
            glm::ivec3 minCoord(bbox.min().x(), bbox.min().y(), bbox.min().z());
            glm::ivec3 maxCoord(bbox.max().x(), bbox.max().y(), bbox.max().z());
            
            // Add padding for future sculpting
            const int padding = 10;
            minCoord -= glm::ivec3(padding);
            maxCoord += glm::ivec3(padding);
            
            // Calculate new dimensions (ensure positive)
            glm::ivec3 size = maxCoord - minCoord + glm::ivec3(1);
            dimensions = glm::uvec3(std::max(0, size.x), std::max(0, size.y), std::max(0, size.z));
        }
    }

    // Convert sparse OpenVDB grid to dense array (needed for Undo/Redo)
    // NEW: Uses actual active bounds, not fixed dimensions
    std::vector<float> toDenseArray() const {
        openvdb::CoordBBox bbox = getActiveBounds();
        if (bbox.empty()) {
            // Empty grid - return minimal array
            return std::vector<float>(1, backgroundValue_);
        }
        
        glm::ivec3 minCoord(bbox.min().x(), bbox.min().y(), bbox.min().z());
        glm::ivec3 maxCoord(bbox.max().x(), bbox.max().y(), bbox.max().z());
        glm::ivec3 size = maxCoord - minCoord + glm::ivec3(1);
        
        std::vector<float> denseData(size.x * size.y * size.z, backgroundValue_);
        
        auto accessor = grid_->getConstAccessor();
        for (int z = 0; z < size.z; ++z) {
            for (int y = 0; y < size.y; ++y) {
                for (int x = 0; x < size.x; ++x) {
                    int worldX = minCoord.x + x;
                    int worldY = minCoord.y + y;
                    int worldZ = minCoord.z + z;
                    size_t index = z * size.x * size.y + y * size.x + x;
                    denseData[index] = accessor.getValue(openvdb::Coord(worldX, worldY, worldZ));
                }
            }
        }
        return denseData;
    }

    // Create sparse grid from dense array (needed for Undo/Redo)
    // NEW: Requires bounding box info to correctly restore data
    void fromDenseArray(const std::vector<float>& denseData, const openvdb::CoordBBox& bbox) {
        grid_->clear();
        
        if (bbox.empty() || denseData.empty()) {
            return;
        }
        
        glm::ivec3 minCoord(bbox.min().x(), bbox.min().y(), bbox.min().z());
        glm::ivec3 maxCoord(bbox.max().x(), bbox.max().y(), bbox.max().z());
        glm::ivec3 size = maxCoord - minCoord + glm::ivec3(1);
        
        if (denseData.size() != static_cast<size_t>(size.x * size.y * size.z)) {
            return; // Size mismatch
        }
        
        auto accessor = grid_->getAccessor();
        for (int z = 0; z < size.z; ++z) {
            for (int y = 0; y < size.y; ++y) {
                for (int x = 0; x < size.x; ++x) {
                    size_t index = z * size.x * size.y + y * size.x + x;
                    float value = denseData[index];
                    
                    // Only store values that differ from background (sparsity optimization)
                    if (std::abs(value - backgroundValue_) > 1e-6f) {
                        int worldX = minCoord.x + x;
                        int worldY = minCoord.y + y;
                        int worldZ = minCoord.z + z;
                        accessor.setValue(openvdb::Coord(worldX, worldY, worldZ), value);
                    }
                }
            }
        }
        
        updateDimensions();
    }

    // LEGACY: Overload for backward compatibility (uses logical dimensions)
    void fromDenseArray(const std::vector<float>& denseData) {
        if (denseData.size() != dimensions.x * dimensions.y * dimensions.z) {
            return; // Size mismatch
        }
        
        grid_->clear();
        
        auto accessor = grid_->getAccessor();
        for (unsigned int z = 0; z < dimensions.z; ++z) {
            for (unsigned int y = 0; y < dimensions.y; ++y) {
                for (unsigned int x = 0; x < dimensions.x; ++x) {
                    size_t index = z * dimensions.x * dimensions.y + y * dimensions.x + x;
                    float value = denseData[index];
                    
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