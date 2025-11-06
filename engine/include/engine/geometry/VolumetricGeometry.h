#pragma once

#include "engine/geometry/IGeometry.h"
#include <memory>

namespace Urbaxio::Engine {

// Forward declaration
struct VoxelGrid;

// IGeometry implementation for voxel-based SDF volumes.
class VolumetricGeometry : public IGeometry {
public:
    VolumetricGeometry(std::unique_ptr<VoxelGrid> grid);
    ~VolumetricGeometry() override;

    // Generates a mesh using Marching Cubes via libigl.
    CadKernel::MeshBuffers getRenderMesh(double detailLevel = 1.0) const override;

    // Direct access to the grid for modification (e.g., sculpting).
    VoxelGrid* getGrid();
    const VoxelGrid* getGrid() const;

private:
    std::unique_ptr<VoxelGrid> grid_;
};

} // namespace Urbaxio::Engine

