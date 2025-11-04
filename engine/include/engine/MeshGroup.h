#pragma once

#include <string>
#include <cstddef> // For size_t

namespace Urbaxio::Engine {

    // Describes a part of a mesh that uses a single material.
    struct MeshGroup {
        // The name of the material to be looked up in the MaterialManager.
        std::string materialName = "Default";
        // The starting index within the object's main index buffer (EBO).
        size_t startIndex = 0;
        // The number of indices this group uses.
        size_t indexCount = 0;
    };

} // namespace Urbaxio::Engine


