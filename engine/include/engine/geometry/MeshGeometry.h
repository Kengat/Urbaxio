#pragma once

#include "engine/geometry/IGeometry.h"
#include <set>
#include <memory>
#include <glm/glm.hpp>

namespace Urbaxio::Engine {

    // Concrete implementation of IGeometry for objects that are only defined
    // by their mesh data (e.g., imported from OBJ files without B-Rep).
    class MeshGeometry : public IGeometry {
    public:
        // Takes ownership of the mesh buffers.
        MeshGeometry(CadKernel::MeshBuffers mesh);
        ~MeshGeometry() override;

        // Movable, but not copyable.
        MeshGeometry(const MeshGeometry&) = delete;
        MeshGeometry& operator=(const MeshGeometry&) = delete;
        MeshGeometry(MeshGeometry&&) = default;
        MeshGeometry& operator=(MeshGeometry&&) = default;

        // Simply returns the stored mesh. detailLevel is ignored.
        CadKernel::MeshBuffers getRenderMesh(double detailLevel = 1.0) const override;

        // Mesh-specific methods (read-only)
        const CadKernel::MeshBuffers& getMesh() const;

        // --- Mesh Operations (for Mesh Mode) ---

        // Mutable access for direct modifications
        CadKernel::MeshBuffers& getMutableMesh();

        // Move vertices by translation vector
        bool moveVertices(const std::set<unsigned int>& vertexIndices, const glm::vec3& translation);

        // Extrude face along normal direction
        // faceVertexIndices: set of vertex indices that belong to the face
        // Returns true on success
        bool extrudeFace(const std::set<unsigned int>& faceVertexIndices,
                         const glm::vec3& normal, float distance);

        // Create a planar face from a closed vertex loop
        // Uses ear-clipping triangulation for convex/concave polygons
        static std::unique_ptr<MeshGeometry> createFromVertexLoop(
            const std::vector<glm::vec3>& vertices, const glm::vec3& normal);

        // Serialization for undo/redo
        std::vector<char> serialize() const;
        static std::unique_ptr<MeshGeometry> deserialize(const std::vector<char>& data);

        // Recalculate normals after mesh modification
        void recalculateNormals();

    private:
        CadKernel::MeshBuffers mesh_;

        // Helper: find boundary edges of a face (edges with only one adjacent triangle in face)
        std::vector<std::pair<unsigned int, unsigned int>> findBoundaryEdges(
            const std::set<unsigned int>& faceVertexIndices) const;

        // Helper: add a vertex and return its index
        unsigned int addVertex(const glm::vec3& pos, const glm::vec3& normal);

        // Helper: add a triangle
        void addTriangle(unsigned int i0, unsigned int i1, unsigned int i2);
    };

} // namespace Urbaxio::Engine
