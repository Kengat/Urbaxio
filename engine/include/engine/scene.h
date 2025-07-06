#ifndef URBAXIO_SCENE_H
#define URBAXIO_SCENE_H

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <map>
#include <utility>
#include <set> // For visited segments in DFS
#include <glm/glm.hpp>

// Forward declare OCCT types to avoid including heavy headers here
class gp_Pln;
class TopoDS_Shape;
class TopoDS_Face;

namespace Urbaxio::Engine { class SceneObject; }

namespace Urbaxio {
    const float SCENE_POINT_EQUALITY_TOLERANCE = 1e-4f;

    struct Vec3Comparator {
        bool operator()(const glm::vec3& a, const glm::vec3& b) const {
            if (std::abs(a.x - b.x) > SCENE_POINT_EQUALITY_TOLERANCE) return a.x < b.x;
            if (std::abs(a.y - b.y) > SCENE_POINT_EQUALITY_TOLERANCE) return a.y < b.y;
            if (std::abs(a.z - b.z) > SCENE_POINT_EQUALITY_TOLERANCE) return a.z < b.z;
            return false;
        }
    };
}

namespace Urbaxio::Engine {

    class Scene {
    public:
        Scene();
        ~Scene();

        Scene(const Scene&) = delete;
        Scene& operator=(const Scene&) = delete;
        Scene(Scene&&) = default;
        Scene& operator=(Scene&&) = default;

        SceneObject* create_object(const std::string& name);
        SceneObject* create_box_object(const std::string& name, double dx, double dy, double dz);
        SceneObject* get_object_by_id(uint64_t id);
        const SceneObject* get_object_by_id(uint64_t id) const;
        std::vector<SceneObject*> get_all_objects();
        std::vector<const SceneObject*> get_all_objects() const;

        void AddUserLine(const glm::vec3& start, const glm::vec3& end);
        const std::vector<std::pair<glm::vec3, glm::vec3>>& GetLineSegments() const;
        void ClearUserLines();

        // --- Geometry Modification ---
        bool ExtrudeFace(uint64_t objectId, const std::vector<size_t>& faceTriangleIndices, const glm::vec3& direction, float distance);

    private:
        std::unordered_map<uint64_t, std::unique_ptr<SceneObject>> objects_;
        uint64_t next_object_id_ = 1;
        int next_face_id_ = 1; // For naming created faces

        std::vector<std::pair<glm::vec3, glm::vec3>> lineSegments_;
        std::vector<bool> segmentUsedInFace_;
        std::map<glm::vec3, std::vector<size_t>, Urbaxio::Vec3Comparator> vertexAdjacency_;

        glm::vec3 MergeVertex(const glm::vec3& p, bool& foundExisting);
        
        // --- Face Creation Logic ---
        void FindAndCreateFaces(size_t newSegmentIndex);
        bool PerformDFS(
            const glm::vec3& startNode,         // The ultimate start of the entire path (e.g., pA of the new line)
            const glm::vec3& currentNode,       // Current vertex in DFS
            const glm::vec3& targetNode,        // The node we are trying to reach (e.g., pA of the new line)
            std::vector<glm::vec3>& currentPathVertices, // Vertices in the current path (excluding startNode initially)
            std::vector<size_t>& currentPathSegmentIndices, // Indices of segments in the path
            std::set<size_t>& visitedSegmentsDFS, // Segments visited in *this specific DFS call*
            size_t originatingSegmentIndex,     // Index of the new segment that triggered this DFS
            int& recursionDepth                 // To prevent infinite loops in complex graphs
        );
        bool ArePointsCoplanar(const std::vector<glm::vec3>& points, gp_Pln& outPlane);
        void CreateOCCTFace(const std::vector<glm::vec3>& orderedVertices, const gp_Pln& plane);
        
        // --- Push/Pull Helpers ---
        TopoDS_Face FindOriginalFace(const TopoDS_Shape& shape, const std::vector<glm::vec3>& faceVertices);
    };

}

#endif // URBAXIO_SCENE_H