#pragma once

namespace Urbaxio::Engine {

    // Controls how new geometry is created and how operations work
    enum class GeometryMode {
        BRep,   // Full OpenCASCADE parametric geometry (default)
        Mesh    // Simplified direct mesh manipulation (SketchUp-like)
    };

} // namespace Urbaxio::Engine
