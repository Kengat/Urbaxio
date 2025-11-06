#pragma once

#include "engine/geometry/IGeometry.h"
#include <cad_kernel/cad_kernel.h> // For OCCT_ShapeUniquePtr

class TopoDS_Shape;


namespace Urbaxio::Engine {

    // Concrete implementation of IGeometry for OpenCASCADE B-Rep shapes.
    class BRepGeometry : public IGeometry {
    public:
        // Takes ownership of the shape.
        BRepGeometry(CadKernel::OCCT_ShapeUniquePtr shape);
        ~BRepGeometry() override;

        // Movable, but not copyable.
        BRepGeometry(const BRepGeometry&) = delete;
        BRepGeometry& operator=(const BRepGeometry&) = delete;
        BRepGeometry(BRepGeometry&&) = default;
        BRepGeometry& operator=(BRepGeometry&&) = default;

        // Generates mesh by triangulating the OCCT shape.
        CadKernel::MeshBuffers getRenderMesh(double detailLevel = 1.0) const override;

        // BRep-specific methods.
        const TopoDS_Shape* getShape() const;
        void setShape(CadKernel::OCCT_ShapeUniquePtr shape);

    private:
        CadKernel::OCCT_ShapeUniquePtr shape_;
    };

} // namespace Urbaxio::Engine
