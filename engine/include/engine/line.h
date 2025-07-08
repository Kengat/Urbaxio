#pragma once

#include <glm/glm.hpp>

namespace Urbaxio::Engine {

    // Represents a single line segment in the scene.
    struct Line {
        glm::vec3 start;
        glm::vec3 end;
        bool usedInFace = false;
        // Other properties like layer, style, etc. can be added later.
    };

} // namespace Urbaxio::Engine 