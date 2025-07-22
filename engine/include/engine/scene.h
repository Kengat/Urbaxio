#ifndef URBAXIO_SCENE_H
#define URBAXIO_SCENE_H

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <map>
#include <utility>
#include <set>
#include <glm/glm.hpp>
#include "engine/line.h"
#include "engine/commands/CommandManager.h"

// Forward declare OCCT types
class gp_Pln;
class gp_Pnt;
class TopoDS_Shape;
class TopoDS_Face;
class TopoDS_Edge;

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

    // Memento for capturing the topological state of the scene.
    // Full definition is here.
    struct SceneState {
        std::map<uint64_t, Line> lines;
        std::map<glm::vec3, std::vector<uint64_t>, Urbaxio::Vec3Comparator> vertexAdjacency;
        std::unordered_map<uint64_t, std::vector<char>> serializedObjects;
        uint64_t nextLineId;
    };

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
        void DeleteObject(uint64_t id);

        // --- Undo/Redo System ---
        CommandManager* getCommandManager();
        std::unique_ptr<SceneState> CaptureState();
        void RestoreState(const SceneState& state);

        // --- Line Management ---
        void AddUserLine(const glm::vec3& start, const glm::vec3& end);
        void ClearUserLines();
        const std::map<uint64_t, Line>& GetAllLines() const;
        glm::vec3 SplitLineAtPoint(uint64_t lineId, const glm::vec3& splitPoint);

        // --- Geometry Modification ---
        bool ExtrudeFace(uint64_t objectId, const std::vector<size_t>& faceTriangleIndices, const glm::vec3& direction, float distance, bool disableMerge = false);

        // --- Geometry Synchronization ---
        void UpdateObjectBoundary(SceneObject* obj);

        // --- Testing Infrastructure ---
        void ClearScene();
        void TestFaceSplitting();

    private:
        std::unordered_map<uint64_t, std::unique_ptr<SceneObject>> objects_;
        uint64_t next_object_id_ = 1;
        int next_face_id_ = 1;

        std::map<uint64_t, Line> lines_;
        uint64_t next_line_id_ = 1;
        std::map<glm::vec3, std::vector<uint64_t>, Urbaxio::Vec3Comparator> vertexAdjacency_;
        
        std::unique_ptr<CommandManager> commandManager_;

        uint64_t AddSingleLineSegment(const glm::vec3& start, const glm::vec3& end);
        void RemoveLine(uint64_t lineId);
        glm::vec3 MergeOrAddVertex(const glm::vec3& p);
        
        std::vector<std::pair<glm::vec3, glm::vec3>> ExtractEdgesFromShape(const TopoDS_Shape& shape);
        
        static bool LineSegmentIntersection(
            const glm::vec3& p1, const glm::vec3& p2,
            const glm::vec3& p3, const glm::vec3& p4,
            glm::vec3& out_intersection_point
        );
        
        void FindAndCreateFaces(uint64_t newLineId);
        bool PerformDFS(
            const glm::vec3& startNode,
            const glm::vec3& currentNode,
            const glm::vec3& targetNode,
            std::vector<glm::vec3>& currentPathVertices,
            std::vector<uint64_t>& currentPathLineIDs,
            std::set<uint64_t>& visitedLinesDFS,
            uint64_t originatingLineId,
            int& recursionDepth
        );
        bool ArePointsCoplanar(const std::vector<glm::vec3>& points, gp_Pln& outPlane);
        void CreateOCCTFace(const std::vector<glm::vec3>& orderedVertices, const gp_Pln& plane);
        
        TopoDS_Face FindOriginalFace(const TopoDS_Shape& shape, const std::vector<glm::vec3>& faceVertices, const glm::vec3& faceNormal);
        void AnalyzeShape(const TopoDS_Shape& shape, const std::string& label);

        SceneObject* CreateRectangularFace(const std::string& name, const gp_Pnt& p1, const gp_Pnt& p2, const gp_Pnt& p3, const gp_Pnt& p4);
    };

}

#endif // URBAXIO_SCENE_H