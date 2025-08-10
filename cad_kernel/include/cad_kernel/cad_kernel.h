#ifndef URBAXIO_CAD_KERNEL_H
#define URBAXIO_CAD_KERNEL_H

#include <cstdint>
#include <memory>
#include "cad_kernel/MeshBuffers.h"

class TopoDS_Shape;

namespace Urbaxio::CadKernel {

    // ---                             OCCT ---
    //                       ShapeDeleter   operator()
    struct ShapeDeleter {
        void operator()(TopoDS_Shape* s) const; //                    .cpp
    };
    using OCCT_ShapeUniquePtr = std::unique_ptr<TopoDS_Shape, ShapeDeleter>;

    // ---              ---
    void initialize();
    OCCT_ShapeUniquePtr create_box(double dx, double dy, double dz);

    // --- Triangulation ---
    MeshBuffers TriangulateShape(const TopoDS_Shape& shape,
        double linDefl = -1.0,  // Linear deflection. If <= 0, will be calculated automatically.
        double angDefl = 0.35,  // Angular deflection in radians (approx. 20 degrees).
        double clampDefl = -1.0); // Optional clamp for auto-calculated linDefl, e.g., 0.05

} // namespace Urbaxio::CadKernel

#endif // URBAXIO_CAD_KERNEL_H