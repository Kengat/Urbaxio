#define GLM_ENABLE_EXPERIMENTAL
#include "engine/scene.h"
#include "engine/scene_object.h"
#include <cad_kernel/cad_kernel.h>
#include <cad_kernel/MeshBuffers.h> // Required for TriangulateShape return type

#include <utility>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <set>
#include <cmath>
#include <algorithm> // for std::reverse

#include <glm/gtx/norm.hpp>
#include <glm/common.hpp> // For epsilonEqual

// OCCT Includes for face creation
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <Standard_Failure.hxx> // For try-catch with OCCT

namespace Urbaxio::Engine {

    const float COPLANARITY_TOLERANCE_SCENE = 1e-4f;
    const int MAX_DFS_DEPTH = 50; // Max recursion depth for DFS to prevent stack overflow

    bool AreVec3Equal(const glm::vec3& a, const glm::vec3& b) {
        return glm::all(glm::epsilonEqual(a, b, Urbaxio::SCENE_POINT_EQUALITY_TOLERANCE));
    }

    Scene::Scene() {}
    Scene::~Scene() = default;

    SceneObject* Scene::create_object(const std::string& name) { uint64_t new_id = next_object_id_++; auto result = objects_.emplace(new_id, std::make_unique<SceneObject>(new_id, name)); if (result.second) { return result.first->second.get(); } else { std::cerr << "Scene: Failed to insert new object with ID " << new_id << " into map." << std::endl; next_object_id_--; return nullptr; } }
    SceneObject* Scene::create_box_object(const std::string& name, double dx, double dy, double dz) { SceneObject* new_obj = create_object(name); if (!new_obj) { return nullptr; } Urbaxio::CadKernel::OCCT_ShapeUniquePtr box_shape_ptr = Urbaxio::CadKernel::create_box(dx, dy, dz); if (!box_shape_ptr) { return nullptr; } const TopoDS_Shape* shape_to_triangulate = box_shape_ptr.get(); if (!shape_to_triangulate || shape_to_triangulate->IsNull()) { return nullptr; } Urbaxio::CadKernel::MeshBuffers mesh_data = Urbaxio::CadKernel::TriangulateShape(*shape_to_triangulate); new_obj->set_shape(std::move(box_shape_ptr)); if (!mesh_data.isEmpty()) { new_obj->set_mesh_buffers(std::move(mesh_data)); } else { std::cerr << "Scene: Warning - Triangulation failed for box '" << name << "'." << std::endl; } return new_obj; }
    SceneObject* Scene::get_object_by_id(uint64_t id) { auto it = objects_.find(id); if (it != objects_.end()) { return it->second.get(); } return nullptr; }
    const SceneObject* Scene::get_object_by_id(uint64_t id) const { auto it = objects_.find(id); if (it != objects_.end()) { return it->second.get(); } return nullptr; }
    std::vector<SceneObject*> Scene::get_all_objects() { std::vector<SceneObject*> result; result.reserve(objects_.size()); for (auto const& [id, obj_ptr] : objects_) { result.push_back(obj_ptr.get()); } return result; }
    std::vector<const SceneObject*> Scene::get_all_objects() const { std::vector<const SceneObject*> result; result.reserve(objects_.size()); for (auto const& [id, obj_ptr] : objects_) { result.push_back(obj_ptr.get()); } return result; }


    glm::vec3 Scene::MergeVertex(const glm::vec3& p, bool& foundExisting) {
        foundExisting = false;
        auto it = vertexAdjacency_.find(p);
        if (it != vertexAdjacency_.end()) {
            foundExisting = true;
            return it->first;
        }
        return p;
    }

    void Scene::AddUserLine(const glm::vec3& start, const glm::vec3& end) {
        bool foundExistingStart, foundExistingEnd;
        glm::vec3 canonicalStart = MergeVertex(start, foundExistingStart);
        glm::vec3 canonicalEnd = MergeVertex(end, foundExistingEnd);

        if (AreVec3Equal(canonicalStart, canonicalEnd)) {
            // std::cout << "Scene: Degenerate line ignored (points too close after merging)." << std::endl;
            return;
        }
        for(const auto& segment : lineSegments_) {
            if ((AreVec3Equal(segment.first, canonicalStart) && AreVec3Equal(segment.second, canonicalEnd)) ||
                (AreVec3Equal(segment.first, canonicalEnd) && AreVec3Equal(segment.second, canonicalStart))) {
                // std::cout << "Scene: Duplicate line segment ignored." << std::endl;
                return;
            }
        }

        lineSegments_.emplace_back(canonicalStart, canonicalEnd);
        segmentUsedInFace_.push_back(false);
        size_t newSegmentIndex = lineSegments_.size() - 1;

        vertexAdjacency_[canonicalStart].push_back(newSegmentIndex);
        vertexAdjacency_[canonicalEnd].push_back(newSegmentIndex);
        
        // std::cout << "Scene: Added line segment " << newSegmentIndex << std::endl;
        FindAndCreateFaces(newSegmentIndex);
    }

    const std::vector<std::pair<glm::vec3, glm::vec3>>& Scene::GetLineSegments() const {
        return lineSegments_;
    }

    void Scene::ClearUserLines() {
        lineSegments_.clear();
        segmentUsedInFace_.clear();
        vertexAdjacency_.clear();
        std::cout << "Scene: Cleared user lines and adjacency data." << std::endl;
    }

    bool Scene::ArePointsCoplanar(const std::vector<glm::vec3>& points, gp_Pln& outPlane) {
        if (points.size() < 3) return false;

        glm::vec3 p0_glm = points[0];
        glm::vec3 p1_glm = glm::vec3(0.0f);
        glm::vec3 p2_glm = glm::vec3(0.0f);
        bool foundP1 = false, foundP2 = false;

        for (size_t i = 1; i < points.size(); ++i) {
            if (!AreVec3Equal(points[i], p0_glm)) {
                p1_glm = points[i];
                foundP1 = true;
                break;
            }
        }
        if (!foundP1) { /*std::cout << "CoplanarCheck: All points coincident with p0." << std::endl;*/ return false; }

        for (size_t i = 1; i < points.size(); ++i) {
            if (!AreVec3Equal(points[i], p0_glm) && !AreVec3Equal(points[i], p1_glm)) {
                glm::vec3 v1_glm = p1_glm - p0_glm;
                glm::vec3 v2_glm = points[i] - p0_glm;
                if (glm::length(glm::cross(v1_glm, v2_glm)) > Urbaxio::SCENE_POINT_EQUALITY_TOLERANCE) { // Check for non-collinearity
                    p2_glm = points[i];
                    foundP2 = true;
                    break;
                }
            }
        }
        if (!foundP2) { /*std::cout << "CoplanarCheck: All points collinear." << std::endl;*/ return false; }

        try {
            gp_Pnt occt_p0(p0_glm.x, p0_glm.y, p0_glm.z);
            gp_Vec v1_occt(p1_glm.x - p0_glm.x, p1_glm.y - p0_glm.y, p1_glm.z - p0_glm.z);
            gp_Vec v2_occt(p2_glm.x - p0_glm.x, p2_glm.y - p0_glm.y, p2_glm.z - p0_glm.z);
            
            gp_Vec normal_vec = v1_occt.Crossed(v2_occt);
            if (normal_vec.Magnitude() < gp::Resolution()) { // Check if normal vector is (close to) zero
                // std::cout << "CoplanarCheck: Degenerate normal vector from cross product." << std::endl;
                return false; 
            }
            gp_Dir occt_normal_dir(normal_vec); // This will normalize
            outPlane = gp_Pln(occt_p0, occt_normal_dir);

            for (const auto& pt_glm : points) {
                gp_Pnt pt_check(pt_glm.x, pt_glm.y, pt_glm.z);
                if (outPlane.Distance(pt_check) > COPLANARITY_TOLERANCE_SCENE) {
                    // std::cout << "CoplanarCheck: Point deviates from plane. Dist: " << outPlane.Distance(pt_check) << std::endl;
                    return false;
                }
            }
        } catch (const Standard_Failure& e) {
            std::cerr << "OCCT Exception during plane creation/check: " << e.GetMessageString() << std::endl;
            return false;
        }
        return true;
    }


    bool Scene::PerformDFS(
        const glm::vec3& pathStartNode, // The very first node of the cycle we are building (e.g. pA of new line)
        const glm::vec3& currentNode,   // Current vertex in DFS traversal
        const glm::vec3& ultimateTargetNode, // The node we are trying to reach to close the cycle (pA)
        std::vector<glm::vec3>& currentPathVertices, // Path so far *excluding* pathStartNode
        std::vector<size_t>& currentPathSegmentIndices,
        std::set<size_t>& visitedSegmentsDFS, // Segments visited in *this specific DFS call*
        size_t originatingSegmentIndex,
        int& recursionDepth
    ) {
        if (recursionDepth++ > MAX_DFS_DEPTH) {
             std::cerr << "DFS max recursion depth reached, aborting path search." << std::endl;
             recursionDepth--;
             return false;
        }

        // Check if we've reached the ultimate target (pathStartNode)
        if (AreVec3Equal(currentNode, ultimateTargetNode)) {
            if (currentPathVertices.size() >= 2) { // Need at least 3 segments in total (new one + 2 from DFS)
                 recursionDepth--;
                 return true; // Cycle found!
            }
        }

        auto it = vertexAdjacency_.find(currentNode);
        if (it == vertexAdjacency_.end()) {
            recursionDepth--;
            return false; // Should not happen if graph is consistent
        }

        const std::vector<size_t>& incidentSegments = it->second;

        // TODO: Implement "left-turn" logic here for sorting incidentSegments based on angle in the plane
        // For now, just iterate in stored order.

        for (size_t segmentIdx : incidentSegments) {
            if (segmentIdx == originatingSegmentIndex || segmentUsedInFace_[segmentIdx] || visitedSegmentsDFS.count(segmentIdx)) {
                continue; // Skip the new line itself, already used segments, or segments in current DFS path
            }

            const auto& segment = lineSegments_[segmentIdx];
            glm::vec3 nextNode = AreVec3Equal(segment.first, currentNode) ? segment.second : segment.first;

            // Tentatively add to path
            currentPathVertices.push_back(nextNode);
            currentPathSegmentIndices.push_back(segmentIdx);
            visitedSegmentsDFS.insert(segmentIdx);

            // Check coplanarity on the fly (only if we have enough points to define a plane)
            bool coplanarCheck = true;
            if (currentPathVertices.size() >= 2) { // currentPathVertices + pathStartNode make up the points for the plane
                std::vector<glm::vec3> planePoints = {pathStartNode};
                planePoints.insert(planePoints.end(), currentPathVertices.begin(), currentPathVertices.end());
                gp_Pln tempPlane; // Not used beyond check here
                if (!ArePointsCoplanar(planePoints, tempPlane)) {
                    coplanarCheck = false;
                }
            }


            if (coplanarCheck) {
                if (PerformDFS(pathStartNode, nextNode, ultimateTargetNode, currentPathVertices, currentPathSegmentIndices, visitedSegmentsDFS, originatingSegmentIndex, recursionDepth)) {
                    recursionDepth--;
                    return true; // Propagate success
                }
            }

            // Backtrack
            visitedSegmentsDFS.erase(segmentIdx);
            currentPathSegmentIndices.pop_back();
            currentPathVertices.pop_back();
        }
        recursionDepth--;
        return false;
    }


    void Scene::CreateOCCTFace(const std::vector<glm::vec3>& orderedVertices, const gp_Pln& plane) {
        if (orderedVertices.size() < 3) return;
        BRepBuilderAPI_MakeWire wireMaker;
        std::vector<TopoDS_Edge> edges;
        try {
            for (size_t i = 0; i < orderedVertices.size(); ++i) {
                gp_Pnt p1_occt(orderedVertices[i].x, orderedVertices[i].y, orderedVertices[i].z);
                gp_Pnt p2_occt(orderedVertices[(i + 1) % orderedVertices.size()].x, orderedVertices[(i + 1) % orderedVertices.size()].y, orderedVertices[(i + 1) % orderedVertices.size()].z);
                TopoDS_Vertex v1 = BRepBuilderAPI_MakeVertex(p1_occt);
                TopoDS_Vertex v2 = BRepBuilderAPI_MakeVertex(p2_occt);
                TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(v1, v2);
                if (edge.IsNull()) {
                    std::cerr << "OCCT Error: Failed to create edge for face." << std::endl;
                    return;
                }
                edges.push_back(edge);
            }
            for(const auto& edge : edges) wireMaker.Add(edge);
        } catch (const Standard_Failure& e) {
            std::cerr << "OCCT Exception during edge/vertex creation for face: " << e.GetMessageString() << std::endl;
            return;
        }

        if (wireMaker.IsDone() && !wireMaker.Wire().IsNull()) {
            TopoDS_Wire wire = wireMaker.Wire();
            BRepBuilderAPI_MakeFace faceMaker(plane, wire, Standard_True);
            if (faceMaker.IsDone() && !faceMaker.Face().IsNull()) {
                TopoDS_Face face = faceMaker.Face();
                std::string face_name = "AutoFace_" + std::to_string(next_face_id_++);
                SceneObject* new_face_obj = create_object(face_name);
                if (new_face_obj) {
                    TopoDS_Shape* shape_copy = new TopoDS_Shape(face);
                    new_face_obj->set_shape(Urbaxio::CadKernel::OCCT_ShapeUniquePtr(shape_copy));
                    Urbaxio::CadKernel::MeshBuffers mesh_data = Urbaxio::CadKernel::TriangulateShape(face);
                    if (!mesh_data.isEmpty()) {
                        new_face_obj->set_mesh_buffers(std::move(mesh_data));
                        // GPU Upload will be handled by the Shell's main loop
                        std::cout << "Scene: Auto-created Face object: " << face_name << ". Mesh ready for GPU." << std::endl;
                    } else {
                        std::cerr << "Scene: Triangulation failed for auto-face " << new_face_obj->get_id() << std::endl;
                    }
                }
            } else {
                std::cerr << "OCCT Error: Failed to create face from wire. Error: " << faceMaker.Error() << std::endl;
            }
        } else {
            std::cerr << "OCCT Error: Failed to create wire for face. Error: " << wireMaker.Error() << std::endl;
        }
    }


    void Scene::FindAndCreateFaces(size_t newSegmentIndex) {
        // std::cout << "DEBUG: Scene::FindAndCreateFaces called for new segment index " << newSegmentIndex << std::endl;
        if (newSegmentIndex >= lineSegments_.size() || segmentUsedInFace_[newSegmentIndex]) {
            return; // Segment invalid or already part of a face
        }

        const auto& newSeg = lineSegments_[newSegmentIndex];
        const glm::vec3& pA = newSeg.first;  // Start of the new segment
        const glm::vec3& pB = newSeg.second; // End of the new segment

        // Try to find a cycle starting from pB and ending at pA (using other lines)
        std::vector<glm::vec3> pathVerticesCollector; // Will store [v1, v2, ..., pA] if path found
        std::vector<size_t> pathSegmentIndicesCollector;
        std::set<size_t> visitedSegmentsInCurrentDFS;
        visitedSegmentsInCurrentDFS.insert(newSegmentIndex); // Don't traverse the new segment back immediately
        int recursionDepth = 0;

        // We start DFS from pB, trying to reach pA. pathStartNode for coplanarity is pA.
        if (PerformDFS(pA, pB, pA, pathVerticesCollector, pathSegmentIndicesCollector, visitedSegmentsInCurrentDFS, newSegmentIndex, recursionDepth)) {
            // pathVerticesCollector should be [neighbor_of_pB, ..., pA]
            
            std::vector<glm::vec3> finalOrderedVertices;
            finalOrderedVertices.push_back(pA); // Start of the new segment that triggered face finding
            finalOrderedVertices.push_back(pB); // End of the new segment

            // Add vertices from the DFS path, excluding the last one if it's pA (which it should be)
            // because pA is already the start of our finalOrderedVertices.
            if (!pathVerticesCollector.empty()) {
                for (size_t i = 0; i < pathVerticesCollector.size() -1; ++i) { // Iterate up to N-1
                    finalOrderedVertices.push_back(pathVerticesCollector[i]);
                }
                // The last element pathVerticesCollector.back() should be pA.
                // If it's not, then the DFS didn't correctly close the loop to pA.
                if (!AreVec3Equal(pathVerticesCollector.back(), pA)) {
                    std::cout << "DEBUG ERROR: DFS path did not end at the start node pA!" << std::endl;
                    return; // Abort face creation
                }
            } else {
                // This means newSegment (pA, pB) was closed by a direct segment (pB, pA)
                // which should have been caught by duplicate segment check earlier if already existing.
                // Or, if pathVerticesCollector is empty, it means pB is directly connected to pA by ONE other segment.
                // DFS(pA, pB, pA, path, ...) -> path would contain just [pA] if pB connected to pA via one segment.
                // In this case, finalOrderedVertices = [pA, pB] and we need one more point.
                // This logic needs refinement if pathVerticesCollector can be empty on success.
                // For now, assume pathVerticesCollector has at least pA if successful.
                 std::cout << "DEBUG: DFS returned success with an empty path collector (should contain at least target)." << std::endl;
                 return;
            }


            std::cout << "DEBUG: DFS found a cycle. Final ordered vertices for OCCT: " << finalOrderedVertices.size() << std::endl;
            for(size_t i=0; i < finalOrderedVertices.size(); ++i) {
                std::cout << "  finalCycleVert[" << i << "]: (" << finalOrderedVertices[i].x << ", " << finalOrderedVertices[i].y << ", " << finalOrderedVertices[i].z << ")" << std::endl;
            }

            if (finalOrderedVertices.size() < 3) {
                std::cout << "DEBUG: Cycle has less than 3 distinct vertices for OCCT wire. Size: " << finalOrderedVertices.size() << std::endl;
                return;
            }

            gp_Pln cyclePlane;
            if (ArePointsCoplanar(finalOrderedVertices, cyclePlane)) {
                std::cout << "DEBUG: Cycle is coplanar. Creating face." << std::endl;
                CreateOCCTFace(finalOrderedVertices, cyclePlane); // Pass finalOrderedVertices

                segmentUsedInFace_[newSegmentIndex] = true;
                for (size_t segIdx : pathSegmentIndicesCollector) {
                    segmentUsedInFace_[segIdx] = true;
                }
            } else {
                std::cout << "DEBUG: Cycle found by DFS is not coplanar." << std::endl;
            }
        } else {
            // std::cout << "DEBUG: DFS did not find a cycle for new segment " << newSegmentIndex << std::endl;
        }
    }

} // namespace Urbaxio::Engine