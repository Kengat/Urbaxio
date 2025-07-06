#pragma once //                             

#include <vector>

namespace Urbaxio::CadKernel {

    //                                                      
    struct MeshBuffers
    {
        std::vector<float> vertices;   //                   (x, y, z, x, y, z, ...)
        std::vector<float> normals;    //                (nx, ny, nz, nx, ny, nz, ...)
        std::vector<unsigned int> indices; //                       (i1, i2, i3, i1, i2, i3, ...)

        //                          
        void clear() {
            vertices.clear();
            normals.clear();
            indices.clear();
        }

        //         ,              
        bool isEmpty() const {
            return vertices.empty() || indices.empty();
        }
    };

} // namespace Urbaxio::CadKernel 