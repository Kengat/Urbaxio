#include "engine/geometry/MeshGeometry.h"
#include <utility>

namespace Urbaxio::Engine {

    MeshGeometry::MeshGeometry(CadKernel::MeshBuffers mesh) : mesh_(std::move(mesh)) {}
    
    MeshGeometry::~MeshGeometry() = default;

    CadKernel::MeshBuffers MeshGeometry::getRenderMesh(double detailLevel) const {
        // Just return a copy of the stored mesh. detailLevel is ignored for this geometry type.
        return mesh_;
    }

    const CadKernel::MeshBuffers& MeshGeometry::getMesh() const {
        return mesh_;
    }

} // namespace Urbaxio::Engine
