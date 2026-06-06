#pragma once

#include <cad_kernel/MeshBuffers.h>
#include <vector>

namespace Urbaxio::Engine {

    struct SubObjectMovePreviewMesh {
        Urbaxio::CadKernel::MeshBuffers mesh;
        std::vector<unsigned int> movingVertexIndices;
    };

} // namespace Urbaxio::Engine
