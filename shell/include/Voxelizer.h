#pragma once

#include <memory>

// Forward declarations to keep this header light
class TopoDS_Shape;

namespace Urbaxio::Engine {
    class VoxelGrid;
}

namespace Urbaxio {

class Voxelizer {
public:
    // A single, static function to perform the conversion.
    // It's self-contained and thread-safe by design.
    static std::unique_ptr<Engine::VoxelGrid> BRepToSDFGrid(const TopoDS_Shape& shape, int resolution);
};

} // namespace Urbaxio

