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

    //                     
    MeshBuffers TriangulateShape(const TopoDS_Shape& shape,
        double linDefl = -1.0,
        double angDefl = 0.35);

} // namespace Urbaxio::CadKernel

#endif // URBAXIO_CAD_KERNEL_H