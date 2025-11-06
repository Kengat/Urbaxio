#pragma once

#include "engine/geometry/IGeometry.h"


namespace Urbaxio::Engine {

    // Concrete implementation of IGeometry for objects that are only defined
    // by their mesh data (e.g., imported from OBJ files without B-Rep).
    class MeshGeometry : public IGeometry {
    public:
        // Takes ownership of the mesh buffers.
        MeshGeometry(CadKernel::MeshBuffers mesh);
        ~MeshGeometry() override;

        // Movable, but not copyable.
        MeshGeometry(const MeshGeometry&) = delete;
        MeshGeometry& operator=(const MeshGeometry&) = delete;
        MeshGeometry(MeshGeometry&&) = default;
        MeshGeometry& operator=(MeshGeometry&&) = default;

        // Simply returns the stored mesh. detailLevel is ignored.
        CadKernel::MeshBuffers getRenderMesh(double detailLevel = 1.0) const override;
        
        // Mesh-specific methods
        const CadKernel::MeshBuffers& getMesh() const;


    private:
        CadKernel::MeshBuffers mesh_;
    };

} // namespace Urbaxio::Engine
