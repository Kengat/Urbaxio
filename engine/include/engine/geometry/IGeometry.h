#pragma once

#include <cad_kernel/MeshBuffers.h>
#include <memory>

namespace Urbaxio::Engine {

    // Abstract base class for all geometry types (B-Rep, Volumetric, Mesh-only, etc.)
    class IGeometry {
    public:
        virtual ~IGeometry() = default;

        // Generates a renderable mesh from the geometry representation.
        // detailLevel can be used by implementations to control mesh density.
        virtual CadKernel::MeshBuffers getRenderMesh(double detailLevel = 1.0) const = 0;
    };

} // namespace Urbaxio::Engine
