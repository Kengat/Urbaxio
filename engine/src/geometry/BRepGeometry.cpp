#include "engine/geometry/BRepGeometry.h"
#include <TopoDS_Shape.hxx>

namespace Urbaxio::Engine {

    BRepGeometry::BRepGeometry(CadKernel::OCCT_ShapeUniquePtr shape) : shape_(std::move(shape)) {}

    BRepGeometry::~BRepGeometry() = default;

    CadKernel::MeshBuffers BRepGeometry::getRenderMesh(double detailLevel) const {
        if (!shape_ || shape_->IsNull()) {
            return {};
        }
        // detailLevel could be used to control triangulation parameters in the future.
        return CadKernel::TriangulateShape(*shape_);
    }

    const TopoDS_Shape* BRepGeometry::getShape() const {
        return shape_.get();
    }
    
    void BRepGeometry::setShape(CadKernel::OCCT_ShapeUniquePtr shape) {
        shape_ = std::move(shape);
    }

} // namespace Urbaxio::Engine
