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
#include <algorithm> // for std::reverse, std::remove

#include <glm/gtx/norm.hpp>
#include <glm/common.hpp> // For epsilonEqual
#include <glm/gtx/intersect.hpp> // For glm::intersectRayPlane

// OCCT Includes for face creation & modification
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Solid.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs.hxx>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepLProp_SLProps.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Splitter.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <ShapeFix_Shape.hxx>
#include <Standard_Failure.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopExp.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>

// --- INCLUDES FOR NEW PARAMETRIC SPLITTING ---
#include <Geom2d_Line.hxx>
#include <ShapeAnalysis_Surface.hxx>
#include <BOPAlgo_Splitter.hxx>
#include <BOPAlgo_Alerts.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Dir2d.hxx>
#include <ShapeBuild_ReShape.hxx> // For tracking sub-shapes after healing
#include <TopTools_ListOfShape.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx> 
#include <ShapeFix_Wire.hxx> // NEW: Include for wire healing
#include <ShapeExtend_Status.hxx> // NEW: Include for status flags


namespace { // Anonymous namespace for utility functions
    bool PointOnLineSegment(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b) {
        const float ON_SEGMENT_TOLERANCE = 1e-4f;
        
        // Quick check if it's one of the endpoints
        if (glm::distance2(p, a) < ON_SEGMENT_TOLERANCE * ON_SEGMENT_TOLERANCE ||
            glm::distance2(p, b) < ON_SEGMENT_TOLERANCE * ON_SEGMENT_TOLERANCE) {
            return false; // It's an endpoint, no split needed
        }

        // Check for collinearity using cross product
        glm::vec3 pa = p - a;
        glm::vec3 ba = b - a;
        if (glm::length2(glm::cross(pa, ba)) > ON_SEGMENT_TOLERANCE * ON_SEGMENT_TOLERANCE) {
            return false; // Not collinear
        }

        // Check if the point is within the segment bounds
        float dot_product = glm::dot(pa, ba);
        if (dot_product < 0.0f || dot_product > glm::length2(ba)) {
            return false; // Outside the segment
        }

        return true;
    }
}

namespace Urbaxio::Engine {

    const float COPLANARITY_TOLERANCE_SCENE = 1e-4f;
    const int MAX_DFS_DEPTH = 50; // Max recursion depth for DFS to prevent stack overflow

    bool AreVec3Equal(const glm::vec3& a, const glm::vec3& b) {
        return glm::all(glm::epsilonEqual(a, b, Urbaxio::SCENE_POINT_EQUALITY_TOLERANCE));
    }

    Scene::Scene() {
        commandManager_ = std::make_unique<CommandManager>();
    }
    Scene::~Scene() = default;

    CommandManager* Scene::getCommandManager() {
        return commandManager_.get();
    }

    SceneObject* Scene::create_object(const std::string& name) { uint64_t new_id = next_object_id_++; auto result = objects_.emplace(new_id, std::make_unique<SceneObject>(new_id, name)); if (result.second) { return result.first->second.get(); } else { std::cerr << "Scene: Failed to insert new object with ID " << new_id << " into map." << std::endl; next_object_id_--; return nullptr; } }
    SceneObject* Scene::create_box_object(const std::string& name, double dx, double dy, double dz) { SceneObject* new_obj = create_object(name); if (!new_obj) { return nullptr; } Urbaxio::CadKernel::OCCT_ShapeUniquePtr box_shape_ptr = Urbaxio::CadKernel::create_box(dx, dy, dz); if (!box_shape_ptr) { return nullptr; } const TopoDS_Shape* shape_to_triangulate = box_shape_ptr.get(); if (!shape_to_triangulate || shape_to_triangulate->IsNull()) { return nullptr; } new_obj->set_shape(std::move(box_shape_ptr)); UpdateObjectBoundary(new_obj); Urbaxio::CadKernel::MeshBuffers mesh_data = Urbaxio::CadKernel::TriangulateShape(*new_obj->get_shape()); if (!mesh_data.isEmpty()) { new_obj->set_mesh_buffers(std::move(mesh_data)); } else { std::cerr << "Scene: Warning - Triangulation failed for box '" << name << "'." << std::endl; } return new_obj; }
    SceneObject* Scene::get_object_by_id(uint64_t id) { auto it = objects_.find(id); if (it != objects_.end()) { return it->second.get(); } return nullptr; }
    const SceneObject* Scene::get_object_by_id(uint64_t id) const { auto it = objects_.find(id); if (it != objects_.end()) { return it->second.get(); } return nullptr; }
    std::vector<SceneObject*> Scene::get_all_objects() { std::vector<SceneObject*> result; result.reserve(objects_.size()); for (auto const& [id, obj_ptr] : objects_) { result.push_back(obj_ptr.get()); } return result; }
    std::vector<const SceneObject*> Scene::get_all_objects() const { std::vector<const SceneObject*> result; result.reserve(objects_.size()); for (auto const& [id, obj_ptr] : objects_) { result.push_back(obj_ptr.get()); } return result; }
    
    // --- Delete Object ---
    void Scene::DeleteObject(uint64_t id) {
        auto it = objects_.find(id);
        if (it == objects_.end()) {
            return;
        }

        SceneObject* obj = it->second.get();

        // Remove the object's boundary lines from the scene
        // Create a copy because RemoveLine modifies the set via UpdateObjectBoundary
        std::set<uint64_t> lines_to_remove = obj->boundaryLineIDs;
        for (uint64_t lineId : lines_to_remove) {
            RemoveLine(lineId);
        }

        // Erase the object from the map, which will call its destructor and free memory
        objects_.erase(it);
        std::cout << "Scene: Deleted object " << id << std::endl;
    }


    // Finds an existing vertex within tolerance or returns the input point.
    // This is the core of vertex merging.
    glm::vec3 Scene::MergeOrAddVertex(const glm::vec3& p) {
        auto it = vertexAdjacency_.find(p);
        if (it != vertexAdjacency_.end()) {
            return it->first; // Return the canonical vertex
        }
        // If not found, it implies a new vertex. The map will be updated when a line is added.
        return p;
    }



    // Public method to add a line.
    void Scene::AddUserLine(const glm::vec3& start, const glm::vec3& end) {
        // --- TRANSACTION LOGIC START ---
        // Store current state to allow for rollback on failure
        auto original_lines = lines_;
        auto original_adjacency = vertexAdjacency_;
        auto original_next_line_id = next_line_id_;

        try {
            // 1. Proactively split existing lines at the new line's endpoints.
            std::vector<std::pair<uint64_t, glm::vec3>> splits_to_perform;
            for (const auto& [line_id, line] : lines_) {
                if (PointOnLineSegment(start, line.start, line.end)) {
                    splits_to_perform.push_back({line_id, start});
                }
                if (PointOnLineSegment(end, line.start, line.end)) {
                    splits_to_perform.push_back({line_id, end});
                }
            }
            for(const auto& split : splits_to_perform) {
                SplitLineAtPoint(split.first, split.second);
            }

            // 2. Find T-junction intersections between the new line segment and existing lines.
            std::map<uint64_t, glm::vec3> intersections_on_existing_lines;
            std::vector<glm::vec3> split_points_on_new_line;

            for (const auto& [existing_id, existing_line] : lines_) {
                glm::vec3 intersection_point;
                if (LineSegmentIntersection(start, end, existing_line.start, existing_line.end, intersection_point)) {
                    intersections_on_existing_lines[existing_id] = intersection_point;
                    split_points_on_new_line.push_back(intersection_point);
                }
            }
            for (const auto& [line_id, point] : intersections_on_existing_lines) {
                SplitLineAtPoint(line_id, point);
            }
            
            // 3. Add the new line, now potentially split by the T-junctions.
            std::vector<glm::vec3> all_points = { start };
            all_points.insert(all_points.end(), split_points_on_new_line.begin(), split_points_on_new_line.end());
            all_points.push_back(end);

            glm::vec3 dir = end - start;
            std::sort(all_points.begin(), all_points.end(), 
                [&start, &dir](const glm::vec3& a, const glm::vec3& b) {
                    if (glm::length2(dir) < 1e-9) return false;
                    return glm::dot(a - start, dir) < glm::dot(b - start, dir);
                });
            all_points.erase(std::unique(all_points.begin(), all_points.end(), AreVec3Equal), all_points.end());

            // 4. Add the new segments and check for new faces for each one.
            std::vector<uint64_t> new_line_ids;
            for (size_t i = 0; i < all_points.size() - 1; ++i) {
                uint64_t new_id = AddSingleLineSegment(all_points[i], all_points[i+1]);
                if (new_id != 0) {
                    new_line_ids.push_back(new_id);
                }
            }
            
            for (uint64_t id : new_line_ids) {
                FindAndCreateFaces(id);
            }
        } catch (...) {
            // --- TRANSACTION ROLLBACK ---
            std::cerr << "An exception occurred during AddUserLine. Rolling back changes." << std::endl;
            lines_ = original_lines;
            vertexAdjacency_ = original_adjacency;
            next_line_id_ = original_next_line_id;
        }
    }

    // This is the internal, "dumb" version of AddUserLine. It only adds a segment and updates adjacency.
    // It does NOT trigger face finding.
    uint64_t Scene::AddSingleLineSegment(const glm::vec3& start, const glm::vec3& end) {
        glm::vec3 canonicalStart = MergeOrAddVertex(start);
        glm::vec3 canonicalEnd = MergeOrAddVertex(end);

        if (AreVec3Equal(canonicalStart, canonicalEnd)) {
            return 0; // Invalid line
        }

        // Check for duplicate lines
        for(const auto& [id, line] : lines_) {
            if ((AreVec3Equal(line.start, canonicalStart) && AreVec3Equal(line.end, canonicalEnd)) ||
                (AreVec3Equal(line.start, canonicalEnd) && AreVec3Equal(line.end, canonicalStart))) {
                return id; // Duplicate line, return existing ID
            }
        }
        
        uint64_t newLineId = next_line_id_++;
        lines_[newLineId] = {canonicalStart, canonicalEnd, false};

        vertexAdjacency_[canonicalStart].push_back(newLineId);
        vertexAdjacency_[canonicalEnd].push_back(newLineId);
        
        return newLineId;
    }

    const std::map<uint64_t, Line>& Scene::GetAllLines() const {
        return lines_;
    }

    void Scene::ClearUserLines() {
        lines_.clear();
        vertexAdjacency_.clear();
        next_line_id_ = 1;
        std::cout << "Scene: Cleared user lines and adjacency data." << std::endl;
    }
    
    void Scene::RemoveLine(uint64_t lineId) {
        auto it = lines_.find(lineId);
        if (it == lines_.end()) return;

        const Line& line = it->second;

        // Remove from start vertex adjacency
        auto& startAdj = vertexAdjacency_[line.start];
        startAdj.erase(std::remove(startAdj.begin(), startAdj.end(), lineId), startAdj.end());
        if (startAdj.empty()) {
            vertexAdjacency_.erase(line.start);
        }

        // Remove from end vertex adjacency
        auto& endAdj = vertexAdjacency_[line.end];
        endAdj.erase(std::remove(endAdj.begin(), endAdj.end(), lineId), endAdj.end());
        if (endAdj.empty()) {
            vertexAdjacency_.erase(line.end);
        }

        // Remove from the main map
        lines_.erase(it);
    }
    
    glm::vec3 Scene::SplitLineAtPoint(uint64_t lineId, const glm::vec3& splitPoint) {
        auto it = lines_.find(lineId);
        if (it == lines_.end()) {
            return splitPoint; // Should not happen
        }

        Line originalLine = it->second;
        glm::vec3 canonicalSplitPoint = MergeOrAddVertex(splitPoint);

        // Check if split point is one of the endpoints
        if (AreVec3Equal(originalLine.start, canonicalSplitPoint) || AreVec3Equal(originalLine.end, canonicalSplitPoint)) {
            return canonicalSplitPoint; // No split needed, just return the merged vertex.
        }

        std::cout << "Scene: Splitting line " << lineId << std::endl;

        // 1. Remove the old line from existence
        RemoveLine(lineId);

        // 2. Add two new lines. Use internal function to avoid recursive intersection checks.
        AddSingleLineSegment(originalLine.start, canonicalSplitPoint);
        AddSingleLineSegment(canonicalSplitPoint, originalLine.end);
        
        // Return the canonical merged vertex for the split point
        return canonicalSplitPoint;
    }

    bool Scene::LineSegmentIntersection(
        const glm::vec3& p1, const glm::vec3& p2, // Segment 1
        const glm::vec3& p3, const glm::vec3& p4, // Segment 2
        glm::vec3& out_intersection_point)
    {
        const float EPSILON = 1e-5f;
        glm::vec3 d1 = p2 - p1;
        glm::vec3 d2 = p4 - p3;

        // --- Robust coplanarity check ---
        glm::vec3 p3_p1 = p3 - p1;
        glm::vec3 plane_normal = glm::cross(d1, p3_p1);
        if (glm::length2(plane_normal) < EPSILON * EPSILON) {
            // All 3 points are collinear. Check if p4 is on the same line.
            if (glm::length2(glm::cross(d1, p4 - p1)) > EPSILON * EPSILON) {
                return false; // Not coplanar/collinear
            }
        } else {
            // Check distance of 4th point to the plane
            float dist = glm::dot(p4 - p1, glm::normalize(plane_normal));
            if (std::abs(dist) > EPSILON) {
                return false; // Not coplanar
            }
        }

        glm::vec3 d1_cross_d2 = glm::cross(d1, d2);
        float d1_cross_d2_lenSq = glm::length2(d1_cross_d2);

        if (d1_cross_d2_lenSq < EPSILON * EPSILON) return false;
        
        float t = glm::dot(glm::cross(p3 - p1, d2), d1_cross_d2) / d1_cross_d2_lenSq;
        if (t < EPSILON || t > 1.0f - EPSILON) return false;

        float u = glm::dot(glm::cross(p3 - p1, d1), d1_cross_d2) / d1_cross_d2_lenSq;
        if (u < EPSILON || u > 1.0f - EPSILON) return false;

        out_intersection_point = p1 + t * d1;
        return true;
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
        if (!foundP1) { return false; }

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
        if (!foundP2) { return false; }

        try {
            gp_Pnt occt_p0(p0_glm.x, p0_glm.y, p0_glm.z);
            gp_Vec v1_occt(p1_glm.x - p0_glm.x, p1_glm.y - p0_glm.y, p1_glm.z - p0_glm.z);
            gp_Vec v2_occt(p2_glm.x - p0_glm.x, p2_glm.y - p0_glm.y, p2_glm.z - p0_glm.z);
            
            gp_Vec normal_vec = v1_occt.Crossed(v2_occt);
            if (normal_vec.Magnitude() < gp::Resolution()) { return false; }
            gp_Dir occt_normal_dir(normal_vec);
            outPlane = gp_Pln(occt_p0, occt_normal_dir);

            for (const auto& pt_glm : points) {
                gp_Pnt pt_check(pt_glm.x, pt_glm.y, pt_glm.z);
                if (outPlane.Distance(pt_check) > COPLANARITY_TOLERANCE_SCENE) { return false; }
            }
        } catch (const Standard_Failure& e) {
            std::cerr << "OCCT Exception during plane creation/check: " << e.GetMessageString() << std::endl;
            return false;
        }
        return true;
    }


    bool Scene::PerformDFS(const glm::vec3& pathStartNode, const glm::vec3& currentNode, const glm::vec3& ultimateTargetNode, std::vector<glm::vec3>& currentPathVertices, std::vector<uint64_t>& currentPathLineIDs, std::set<uint64_t>& visitedLinesDFS, uint64_t originatingLineId, int& recursionDepth) {
        if (recursionDepth++ > MAX_DFS_DEPTH) { recursionDepth--; return false; }

        if (AreVec3Equal(currentNode, ultimateTargetNode)) {
            if (currentPathVertices.size() >= 2) { recursionDepth--; return true; }
        }

        auto it = vertexAdjacency_.find(currentNode);
        if (it == vertexAdjacency_.end()) { recursionDepth--; return false; }

        const std::vector<uint64_t>& incidentLines = it->second;

        for (uint64_t lineId : incidentLines) {
            auto line_it = lines_.find(lineId);
            if (line_it == lines_.end()) continue; // Should not happen

            if (lineId == originatingLineId || visitedLinesDFS.count(lineId)) {
                continue;
            }

            const auto& line = line_it->second;
            glm::vec3 nextNode = AreVec3Equal(line.start, currentNode) ? line.end : line.start;

            currentPathVertices.push_back(nextNode);
            currentPathLineIDs.push_back(lineId);
            visitedLinesDFS.insert(lineId);

            bool coplanarCheck = true;
            if (currentPathVertices.size() >= 2) {
                std::vector<glm::vec3> planePoints = {pathStartNode};
                planePoints.insert(planePoints.end(), currentPathVertices.begin(), currentPathVertices.end());
                gp_Pln tempPlane;
                if (!ArePointsCoplanar(planePoints, tempPlane)) { coplanarCheck = false; }
            }

            if (coplanarCheck) {
                if (PerformDFS(pathStartNode, nextNode, ultimateTargetNode, currentPathVertices, currentPathLineIDs, visitedLinesDFS, originatingLineId, recursionDepth)) {
                    recursionDepth--;
                    return true;
                }
            }
            
            visitedLinesDFS.erase(lineId);
            currentPathLineIDs.pop_back();
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
                if (p1_occt.IsEqual(p2_occt, SCENE_POINT_EQUALITY_TOLERANCE)) continue;
                TopoDS_Vertex v1 = BRepBuilderAPI_MakeVertex(p1_occt);
                TopoDS_Vertex v2 = BRepBuilderAPI_MakeVertex(p2_occt);
                TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(v1, v2);
                if (edge.IsNull()) { std::cerr << "OCCT Error: Failed to create edge for face." << std::endl; return; }
                edges.push_back(edge);
            }
            for(const auto& edge : edges) wireMaker.Add(edge);
        } catch (const Standard_Failure& e) { std::cerr << "OCCT Exception during edge/vertex creation for face: " << e.GetMessageString() << std::endl; return; }

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
                    UpdateObjectBoundary(new_face_obj); // <-- Sync lines
                    Urbaxio::CadKernel::MeshBuffers mesh_data = Urbaxio::CadKernel::TriangulateShape(*new_face_obj->get_shape());
                    if (!mesh_data.isEmpty()) {
                        new_face_obj->set_mesh_buffers(std::move(mesh_data));
                        std::cout << "Scene: Auto-created Face object: " << face_name << ". Mesh ready for GPU." << std::endl;
                    } else { std::cerr << "Scene: Triangulation failed for auto-face " << new_face_obj->get_id() << std::endl; }
                }
            } else { std::cerr << "OCCT Error: Failed to create face from wire. Error: " << faceMaker.Error() << std::endl; }
        } else { std::cerr << "OCCT Error: Failed to create wire for face. Error: " << wireMaker.Error() << std::endl; }
    }


    void Scene::FindAndCreateFaces(uint64_t newLineId) {
        auto line_it = lines_.find(newLineId);
        if (line_it == lines_.end() || line_it->second.usedInFace) {
            return;
        }
        
        const Line& newLine = line_it->second;
        const glm::vec3& pA = newLine.start;
        const glm::vec3& pB = newLine.end;
        
        std::vector<glm::vec3> pathVerticesCollector;
        std::vector<uint64_t> pathLineIDsCollector;
        std::set<uint64_t> visitedLinesInCurrentDFS;
        visitedLinesInCurrentDFS.insert(newLineId);
        int recursionDepth = 0;

        if (!PerformDFS(pA, pB, pA, pathVerticesCollector, pathLineIDsCollector, visitedLinesInCurrentDFS, newLineId, recursionDepth)) {
            return;
        }
        
        std::vector<glm::vec3> finalOrderedVertices;
        finalOrderedVertices.push_back(pA);
        finalOrderedVertices.push_back(pB);
        if (!pathVerticesCollector.empty()) {
            for (size_t i = 0; i < pathVerticesCollector.size() - 1; ++i) {
                finalOrderedVertices.push_back(pathVerticesCollector[i]);
            }
        }
        if (finalOrderedVertices.size() < 3) return;

        SceneObject* hostObject = nullptr;
        TopoDS_Face originalHostFace;
        
        gp_Pln cyclePlane;
        if (ArePointsCoplanar(finalOrderedVertices, cyclePlane)) {
            gp_Dir normalDir = cyclePlane.Axis().Direction();
            glm::vec3 loopNormal(normalDir.X(), normalDir.Y(), normalDir.Z());
            
            for (const auto& [id, obj_ptr] : objects_) {
                if (obj_ptr && obj_ptr->has_shape()) {
                    TopoDS_Face foundFace = FindOriginalFace(*obj_ptr->get_shape(), finalOrderedVertices, loopNormal);
                    if (!foundFace.IsNull()) {
                        hostObject = obj_ptr.get();
                        originalHostFace = foundFace;
                        break; 
                    }
                }
            }
        }

        if (hostObject && !originalHostFace.IsNull()) {
            std::cout << "Scene: Detected new loop on existing face of object " << hostObject->get_id() << ". Attempting PARAMETRIC split." << std::endl;
            try {
                ShapeFix_Shape shapeFixer(*hostObject->get_shape());
                shapeFixer.Perform();
                TopoDS_Shape healedShape = shapeFixer.Shape();
                
                Handle(ShapeBuild_ReShape) context = shapeFixer.Context();
                TopoDS_Shape healedFaceShape = context->Apply(originalHostFace);
                if (healedFaceShape.IsNull() || healedFaceShape.ShapeType() != TopAbs_FACE) {
                     throw Standard_Failure("Could not re-find host face after healing.");
                }
                TopoDS_Face healedHostFace = TopoDS::Face(healedFaceShape);
                
                Handle(Geom_Surface) hostSurface = BRep_Tool::Surface(healedHostFace);
                if (hostSurface.IsNull()) {
                    throw Standard_Failure("Host face has no underlying surface.");
                }
                Handle(ShapeAnalysis_Surface) analysis = new ShapeAnalysis_Surface(hostSurface);

                // --- Step 2: Create a list of topologically sound edges ---
                TopTools_ListOfShape edgeList;
                Standard_Real umin, umax, vmin, vmax;
                hostSurface->Bounds(umin, umax, vmin, vmax);

                for (size_t i = 0; i < finalOrderedVertices.size(); ++i) {
                    const glm::vec3& v1_glm = finalOrderedVertices[i];
                    const glm::vec3& v2_glm = finalOrderedVertices[(i + 1) % finalOrderedVertices.size()];
                    
                    gp_Pnt p1_3d(v1_glm.x, v1_glm.y, v1_glm.z);
                    gp_Pnt p2_3d(v2_glm.x, v2_glm.y, v2_glm.z);
                    if (p1_3d.IsEqual(p2_3d, Precision::Confusion())) continue;
                    
                    gp_Pnt2d uv1 = analysis->ValueOfUV(p1_3d, Precision::Confusion());
                    gp_Pnt2d uv2 = analysis->ValueOfUV(p2_3d, Precision::Confusion());

                    // Clamp UV coordinates to surface bounds to avoid floating point errors at the boundary
                    uv1.SetX(std::max(umin, std::min(umax, uv1.X())));
                    uv1.SetY(std::max(vmin, std::min(vmax, uv1.Y())));
                    uv2.SetX(std::max(umin, std::min(umax, uv2.X())));
                    uv2.SetY(std::max(vmin, std::min(vmax, uv2.Y())));
                    
                    if (uv1.IsEqual(uv2, Precision::Confusion())) continue;

                    Handle(Geom2d_Line) pcurve = new Geom2d_Line(uv1, gp_Dir2d(uv2.X() - uv1.X(), uv2.Y() - uv1.Y()));
                    // Use the constructor that takes parameters
                    Standard_Real u_start = 0.0;
                    Standard_Real u_end = uv1.Distance(uv2);
                    BRepBuilderAPI_MakeEdge edgeMaker(pcurve, hostSurface, u_start, u_end);
                    if (!edgeMaker.IsDone()) {
                         std::stringstream ss;
                         ss << "BRepBuilderAPI_MakeEdge failed for segment " << i << ". "
                            << "From UV(" << uv1.X() << ", " << uv1.Y() << ") to UV(" << uv2.X() << ", " << uv2.Y() << ").";
                         throw Standard_Failure(ss.str().c_str());
                    }
                    edgeList.Append(edgeMaker.Edge());
                }
                
                // --- Step 3: Reliable Wire Assembly using ShapeFix_Wire ---
                BRepBuilderAPI_MakeWire tempWireMaker;
                tempWireMaker.Add(edgeList);

                if (tempWireMaker.Wire().IsNull()) {
                    throw Standard_Failure("BRepBuilderAPI_MakeWire failed to create even a disconnected wire.");
                }

                Handle(ShapeFix_Wire) wireFixer = new ShapeFix_Wire();
                wireFixer->Init(tempWireMaker.Wire(), healedHostFace, Precision::Confusion());
                if (!wireFixer->Perform()) {
                    throw Standard_Failure("ShapeFix_Wire::Perform() returned false, indicating a critical failure.");
                }

                if (wireFixer->LastFixStatus(ShapeExtend_FAIL)) {
                    throw Standard_Failure("ShapeFix_Wire could not fix all problems in the wire (e.g., gaps are too large).");
                }

                TopoDS_Wire cuttingWire = wireFixer->Wire();
                if (cuttingWire.IsNull() || !BRep_Tool::IsClosed(cuttingWire)) {
                    throw Standard_Failure("The final wire created by ShapeFix_Wire is not a single, closed loop.");
                }

                // --- CORRECTED SPLITTER LOGIC ---
                BOPAlgo_Splitter splitter;
                
                TopTools_ListOfShape arguments;
                arguments.Append(healedShape);
                splitter.SetArguments(arguments);

                TopTools_ListOfShape tools;
                tools.Append(cuttingWire);
                splitter.SetTools(tools);

                splitter.Perform();

                if (splitter.HasErrors()) {
                    splitter.GetReport()->Dump(std::cout);
                    throw Standard_Failure("BOPAlgo_Splitter failed to perform operation.");
                }
                hostObject->set_shape(Urbaxio::CadKernel::OCCT_ShapeUniquePtr(new TopoDS_Shape(splitter.Shape())));
                UpdateObjectBoundary(hostObject);
                hostObject->set_mesh_buffers(Urbaxio::CadKernel::TriangulateShape(*hostObject->get_shape()));
                hostObject->vao = 0;
                std::cout << "Scene: Object " << hostObject->get_id() << " was successfully split and updated." << std::endl;
            } catch (const Standard_Failure& e) {
                std::cerr << "OCCT Exception during parametric split: " << e.GetMessageString() << std::endl;
            }
        } else {
            // Fallback: This loop was not on an existing face. Create a new one.
            // This is the logic for creating faces from sketches in empty space.
            std::cout << "FindAndCreateFaces: Loop detected, but no suitable host face found. Creating a new independent face object." << std::endl;
            CreateOCCTFace(finalOrderedVertices, cyclePlane);
            lines_.at(newLineId).usedInFace = true;
            for (uint64_t lineId : pathLineIDsCollector) {
                lines_.at(lineId).usedInFace = true;
            }
        }
    }
    
    // --- HELPER FUNCTIONS ---
    std::vector<std::pair<glm::vec3, glm::vec3>> Scene::ExtractEdgesFromShape(const TopoDS_Shape& shape) {
        std::vector<std::pair<glm::vec3, glm::vec3>> edges;
        TopExp_Explorer explorer(shape, TopAbs_EDGE);
        for (; explorer.More(); explorer.Next()) {
            const TopoDS_Edge& edge = TopoDS::Edge(explorer.Current());
            TopoDS_Vertex v1, v2;
            TopExp::Vertices(edge, v1, v2);
            if (v1.IsNull() || v2.IsNull()) continue;
            gp_Pnt p1 = BRep_Tool::Pnt(v1);
            gp_Pnt p2 = BRep_Tool::Pnt(v2);
            edges.push_back({
                { (float)p1.X(), (float)p1.Y(), (float)p1.Z() },
                { (float)p2.X(), (float)p2.Y(), (float)p2.Z() }
            });
        }
        return edges;
    }

    void Scene::UpdateObjectBoundary(SceneObject* obj) {
        if (!obj || !obj->has_shape()) return;
        
        auto new_edges = ExtractEdgesFromShape(*obj->get_shape());
        std::set<uint64_t> new_boundary_line_ids;
        std::set<uint64_t> old_line_ids_to_check = obj->boundaryLineIDs;

        // Add/update lines from new edges
        for (const auto& edge : new_edges) {
            uint64_t line_id = AddSingleLineSegment(edge.first, edge.second);
            if (line_id != 0) {
                lines_[line_id].usedInFace = true;
                new_boundary_line_ids.insert(line_id);
                if(old_line_ids_to_check.count(line_id)) {
                    old_line_ids_to_check.erase(line_id);
                }
            }
        }

        for (uint64_t dead_line_id : old_line_ids_to_check) {
            RemoveLine(dead_line_id);
        }
        
        obj->boundaryLineIDs = new_boundary_line_ids;
    }
    


    // --- PUSH/PULL (MODIFIED) ---
    TopoDS_Face Scene::FindOriginalFace(const TopoDS_Shape& shape, const std::vector<glm::vec3>& faceVertices, const glm::vec3& faceNormal) {
        if (faceVertices.empty()) return TopoDS_Face();
        
        // Find center of user-provided points
        glm::vec3 targetCenter(0.0f);
        for(const auto& v : faceVertices) {
            targetCenter += v;
        }
        targetCenter /= (float)faceVertices.size();
        
        struct FaceCandidate {
            TopoDS_Face face;
            float distance = std::numeric_limits<float>::max();
        };
        
        std::vector<FaceCandidate> candidates;

        TopExp_Explorer faceExplorer(shape, TopAbs_FACE);
        for (; faceExplorer.More(); faceExplorer.Next()) {
            TopoDS_Face candidateFace = TopoDS::Face(faceExplorer.Current());
            
            try {
                BRepAdaptor_Surface surfaceAdaptor(candidateFace, Standard_False);
                if(surfaceAdaptor.GetType() != GeomAbs_Plane) continue;

                // Check if all loop points are on or inside the face
                bool allPointsOnFace = true;
                for(const auto& v : faceVertices) {
                    BRepClass_FaceClassifier classifier;
                    // Use a slightly larger tolerance for classification
                    classifier.Perform(candidateFace, gp_Pnt(v.x, v.y, v.z), 1e-4); 
                    TopAbs_State state = classifier.State();
                    if(state != TopAbs_ON && state != TopAbs_IN) {
                        allPointsOnFace = false;
                        break;
                    }
                }
                
                if (allPointsOnFace) {
                    // This is a valid candidate, calculate its distance to the target
                    GProp_GProps props;
                    BRepGProp::SurfaceProperties(candidateFace, props);
                    gp_Pnt center = props.CentreOfMass();
                    float dist = (float)center.Distance(gp_Pnt(targetCenter.x, targetCenter.y, targetCenter.z));
                    candidates.push_back({candidateFace, dist});
                }
            } catch (const Standard_Failure&) {
                continue;
            }
        }
        
        if (candidates.empty()) {
            return TopoDS_Face();
        }
        
        // Sort candidates by distance, closest first
        std::sort(candidates.begin(), candidates.end(), [](const FaceCandidate& a, const FaceCandidate& b) {
            return a.distance < b.distance;
        });
        
        return candidates[0].face;
    }

    void Scene::AnalyzeShape(const TopoDS_Shape& shape, const std::string& label) {
        if (shape.IsNull()) {
             std::cout << "=== SHAPE ANALYSIS: " << label << " (SHAPE IS NULL) ===" << std::endl;
             return;
        }
        std::cout << "=== SHAPE ANALYSIS: " << label << " ===" << std::endl;
        
        // Basic info
        std::cout << "Shape Type: " << shape.ShapeType() << " (0=Compound, 1=CompSolid, 2=Solid, 3=Shell, 4=Face, 5=Wire, 6=Edge, 7=Vertex)" << std::endl;
        
        // Count components
        TopExp_Explorer solidExp(shape, TopAbs_SOLID);
        int solidCount = 0;
        for (; solidExp.More(); solidExp.Next()) solidCount++;
        
        TopExp_Explorer faceExp(shape, TopAbs_FACE);
        int faceCount = 0;
        for (; faceExp.More(); faceExp.Next()) faceCount++;
        
        std::cout << "Contains: " << solidCount << " solids, " << faceCount << " faces" << std::endl;
        
        // Bounding box
        try {
            Bnd_Box boundingBox;
            BRepBndLib::Add(shape, boundingBox);
            if (!boundingBox.IsVoid()) {
                Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
                boundingBox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
                std::cout << "Bounding Box: (" << xmin << ", " << ymin << ", " << zmin << ") to (" 
                         << xmax << ", " << ymax << ", " << zmax << ")" << std::endl;
                std::cout << "Dimensions: " << (xmax-xmin) << " x " << (ymax-ymin) << " x " << (zmax-zmin) << std::endl;
            } else {
                std::cout << "Bounding Box: VOID" << std::endl;
            }
        } catch (const Standard_Failure& e) {
            std::cout << "Bounding Box: ERROR - " << e.GetMessageString() << std::endl;
        }
        
        // Volume/properties
        try {
            GProp_GProps props;
            BRepGProp::VolumeProperties(shape, props);
            Standard_Real volume = props.Mass();
            gp_Pnt center = props.CentreOfMass();
            std::cout << "Volume: " << volume << std::endl;
            std::cout << "Center of Mass: (" << center.X() << ", " << center.Y() << ", " << center.Z() << ")" << std::endl;
        } catch (const Standard_Failure& e) {
            std::cout << "Volume Properties: ERROR - " << e.GetMessageString() << std::endl;
        }
        
        // Validity
        try {
            BRepCheck_Analyzer analyzer(shape);
            std::cout << "Shape Valid: " << (analyzer.IsValid() ? "YES" : "NO") << std::endl;
        } catch (const Standard_Failure& e) {
            std::cout << "Validity Check: ERROR - " << e.GetMessageString() << std::endl;
        }
        
        std::cout << "=== END ANALYSIS ===" << std::endl << std::endl;
    }

    
    bool Scene::ExtrudeFace(uint64_t objectId, const std::vector<size_t>& faceTriangleIndices, const glm::vec3& direction, float distance, bool disableMerge) {
        SceneObject* obj = get_object_by_id(objectId);
        if (!obj || !obj->has_shape() || faceTriangleIndices.empty() || std::abs(distance) < 1e-4) {
            return false;
        }

        const auto& mesh = obj->get_mesh_buffers();
        const TopoDS_Shape* originalShape = obj->get_shape();
        AnalyzeShape(*originalShape, "ORIGINAL SHAPE");

        std::set<unsigned int> faceVertexIndicesSet;
        for (size_t baseIdx : faceTriangleIndices) {
            faceVertexIndicesSet.insert(mesh.indices[baseIdx]);
            faceVertexIndicesSet.insert(mesh.indices[baseIdx + 1]);
            faceVertexIndicesSet.insert(mesh.indices[baseIdx + 2]);
        }
        std::vector<glm::vec3> faceVertices;
        for (unsigned int idx : faceVertexIndicesSet) {
            faceVertices.push_back({mesh.vertices[idx * 3], mesh.vertices[idx * 3 + 1], mesh.vertices[idx * 3 + 2]});
        }
        
        TopoDS_Face faceToExtrude = FindOriginalFace(*originalShape, faceVertices, direction);
        if (faceToExtrude.IsNull()) {
            std::cerr << "ExtrudeFace Error: Could not find corresponding B-Rep face." << std::endl;
            return false;
        }

        try {
            // --- CORRECTED LOGIC ---
            // 1. Get the mathematical normal of the face's underlying surface
            BRepAdaptor_Surface surfaceAdaptor(faceToExtrude, Standard_True);
            gp_Pln plane = surfaceAdaptor.Plane();
            gp_Dir occtFaceNormal = plane.Axis().Direction();
            
            // 2. Adjust for the face's orientation
            if (faceToExtrude.Orientation() == TopAbs_REVERSED) {
                occtFaceNormal.Reverse();
            }

            // 3. Compare with the user's intended direction
            gp_Dir userDirection(direction.x, direction.y, direction.z);
            float dotProductWithUserDir = occtFaceNormal.Dot(userDirection);

            // 4. Create the final extrusion vector. If user direction is opposite to the face normal,
            // we must extrude in the opposite direction of the normal to achieve the user's goal.
            gp_Vec extrudeVector(occtFaceNormal.XYZ());
            if (dotProductWithUserDir < 0) {
                extrudeVector.Reverse();
            }
            extrudeVector *= distance;
            
            BRepPrimAPI_MakePrism prismMaker(faceToExtrude, extrudeVector);
            
            if (!prismMaker.IsDone()) {
                std::cerr << "OCCT Error: Failed to create prism from face." << std::endl;
                return false;
            }
            TopoDS_Shape prismShape = prismMaker.Shape();
            AnalyzeShape(prismShape, "CREATED PRISM");
            
            TopoDS_Shape finalShape;
            
            // --- UNIVERSAL 3D BODY DETECTION ---
            bool is3DBody = (originalShape->ShapeType() == TopAbs_SOLID || originalShape->ShapeType() == TopAbs_COMPSOLID);
            if (!is3DBody && originalShape->ShapeType() == TopAbs_COMPOUND) {
                TopExp_Explorer ex(*originalShape, TopAbs_SOLID);
                if (ex.More()) is3DBody = true;
            }
            
            if (is3DBody) {
                BRepAdaptor_Surface surfaceAdaptor(faceToExtrude, Standard_True);
                gp_Pln plane = surfaceAdaptor.Plane();
                gp_Dir occtFaceNormal = plane.Axis().Direction();
                if (faceToExtrude.Orientation() == TopAbs_REVERSED) {
                    occtFaceNormal.Reverse();
                }
                double dotProduct = extrudeVector.Dot(gp_Vec(occtFaceNormal));
                std::cout << "Scene: Push/Pull on 3D Body. Normal dot extrusion vector = " << dotProduct << std::endl;
                if (dotProduct >= -1e-6) {
                    std::cout << "Scene: Push/Pull - Detected outward extrusion. Using Fuse operation." << std::endl;
                    BRepAlgoAPI_Fuse fuseAlgo(*originalShape, prismShape);
                    if (!fuseAlgo.IsDone()) { std::cerr << "OCCT Error: BRepAlgoAPI_Fuse failed." << std::endl; return false; }
                    finalShape = fuseAlgo.Shape();
                } else {
                    std::cout << "Scene: Push/Pull - Detected inward extrusion. Using Cut operation." << std::endl;
                    BRepAlgoAPI_Cut cutAlgo(*originalShape, prismShape);
                    if (!cutAlgo.IsDone()) { std::cerr << "OCCT Error: BRepAlgoAPI_Cut failed." << std::endl; return false; }
                    finalShape = cutAlgo.Shape();
                }
                obj->set_shape(Urbaxio::CadKernel::OCCT_ShapeUniquePtr(new TopoDS_Shape(finalShape)));
            } else {
                finalShape = prismShape;
                std::cout << "Scene: Push/Pull - 2D shape converted to solid." << std::endl;
                obj->set_shape(Urbaxio::CadKernel::OCCT_ShapeUniquePtr(new TopoDS_Shape(finalShape)));
            }
            // --- HEALING AND UPDATING ---
            if (obj && obj->has_shape()) {
                ShapeFix_Shape shapeFixer(*obj->get_shape());
                shapeFixer.Perform();
                TopoDS_Shape healedShape = shapeFixer.Shape();
                if (!disableMerge) {
                    ShapeUpgrade_UnifySameDomain Unifier;
                    Unifier.Initialize(healedShape);
                    Unifier.SetLinearTolerance(1e-4); 
                    Unifier.SetAngularTolerance(1e-4); 
                    Unifier.AllowInternalEdges(Standard_True);
                    Unifier.Build();
                    healedShape = Unifier.Shape();
                }
                obj->set_shape(Urbaxio::CadKernel::OCCT_ShapeUniquePtr(new TopoDS_Shape(healedShape)));
                UpdateObjectBoundary(obj);
                obj->set_mesh_buffers(Urbaxio::CadKernel::TriangulateShape(*obj->get_shape()));
                obj->vao = 0;
            }
            std::cout << "Scene: Push/Pull successful. Object " << objectId << " updated." << std::endl;
        } catch (const Standard_Failure& e) {
            std::cerr << "OCCT Exception during ExtrudeFace: " << e.GetMessageString() << std::endl;
            return false;
        }
        return true;
    }

    // --- NEW: Testing Infrastructure ---
    
    void Scene::ClearScene() {
        // This clears the engine's state. The shell is responsible for cleaning up GPU resources.
        objects_.clear();
        next_object_id_ = 1;
        next_face_id_ = 1;
        
        lines_.clear();
        vertexAdjacency_.clear();
        next_line_id_ = 1;
        
        commandManager_->ClearHistory(); // <-- NEW

        std::cout << "Scene: Cleared all objects, lines and command history from engine state." << std::endl;
    }
    
    // Helper to create a simple rectangular face object for testing.
    SceneObject* Scene::CreateRectangularFace(
        const std::string& name,
        const gp_Pnt& p1, const gp_Pnt& p2, const gp_Pnt& p3, const gp_Pnt& p4)
    {
        try {
            TopoDS_Edge edge1 = BRepBuilderAPI_MakeEdge(p1, p2);
            TopoDS_Edge edge2 = BRepBuilderAPI_MakeEdge(p2, p3);
            TopoDS_Edge edge3 = BRepBuilderAPI_MakeEdge(p3, p4);
            TopoDS_Edge edge4 = BRepBuilderAPI_MakeEdge(p4, p1);

            BRepBuilderAPI_MakeWire wireMaker(edge1, edge2, edge3, edge4);
            if (!wireMaker.IsDone()) {
                std::cerr << "CreateRectangularFace Error: Failed to create wire." << std::endl;
                return nullptr;
            }
            TopoDS_Wire wire = wireMaker.Wire();
            BRepBuilderAPI_MakeFace faceMaker(wire, Standard_True);
            if (!faceMaker.IsDone() || faceMaker.Face().IsNull()) {
                std::cerr << "CreateRectangularFace Error: Failed to create face." << std::endl;
                return nullptr;
            }

            TopoDS_Face face = faceMaker.Face();
            SceneObject* new_face_obj = this->create_object(name);
            if (new_face_obj) {
                new_face_obj->set_shape(Urbaxio::CadKernel::OCCT_ShapeUniquePtr(new TopoDS_Shape(face)));
                UpdateObjectBoundary(new_face_obj);
                auto mesh_data = Urbaxio::CadKernel::TriangulateShape(*new_face_obj->get_shape());
                if (!mesh_data.isEmpty()) {
                    new_face_obj->set_mesh_buffers(std::move(mesh_data));
                    std::cout << "Scene: Created test face object: " << name << std::endl;
                }
                return new_face_obj;
            }
        } catch (const Standard_Failure& e) {
            std::cerr << "OCCT Exception in CreateRectangularFace: " << e.GetMessageString() << std::endl;
        }
        return nullptr;
    }
    
    void Scene::TestFaceSplitting() {
        std::cout << "\n\n--- RUNNING FACE SPLITTING TEST ---\n" << std::endl;
        ClearScene();

        // --- Test Setup: Create a large face to draw on ---
        gp_Pnt p1(0, 0, 0);
        gp_Pnt p2(100, 0, 0);
        gp_Pnt p3(100, 100, 0);
        gp_Pnt p4(0, 100, 0);
        
        SceneObject* testFace = CreateRectangularFace("TestFace_Base", p1, p2, p3, p4);
        if (!testFace) {
            std::cerr << "TEST SETUP FAILED: Could not create initial test face." << std::endl;
            return;
        }
        std::cout << "Initial face created. Object count: " << objects_.size() << ". Initial lines: " << lines_.size() << std::endl;

        std::cout << "\n--- SCENARIO: Drawing a 'П' shape on the face ---\n" << std::endl;
        
        // These are the points of the 'П' shape
        glm::vec3 vA(20.0f, 0.0f, 0.0f);   // Start on bottom edge
        glm::vec3 vB(20.0f, 50.0f, 0.0f);  // Go up
        glm::vec3 vC(80.0f, 50.0f, 0.0f);  // Go right
        glm::vec3 vD(80.0f, 0.0f, 0.0f);   // Go down to bottom edge

        // We simulate a user drawing three separate lines.
        // Each call to AddUserLine is a complete user action (click-drag-release or click-move-click).
        // The engine's internal logic should handle the intermediate splits and the final loop detection.
        
        std::cout << "Step 1: Drawing line from edge to inside (A->B)..." << std::endl;
        AddUserLine(vA, vB);
        std::cout << "  Objects after Step 1: " << objects_.size() << ". Lines: " << lines_.size() << std::endl;


        std::cout << "Step 2: Drawing line from point to point (B->C)..." << std::endl;
        AddUserLine(vB, vC);
        std::cout << "  Objects after Step 2: " << objects_.size() << ". Lines: " << lines_.size() << std::endl;


        std::cout << "Step 3: Drawing line from point to edge (C->D) to close the loop..." << std::endl;
        AddUserLine(vC, vD);
        std::cout << "  Objects after Step 3: " << objects_.size() << ". Lines: " << lines_.size() << std::endl;


        // --- FINAL VERIFICATION ---
        // After the three lines are drawn, a 'П' shape is formed. The bottom of the 'П'
        // is closed by the existing edge of the original face.
        // The expected outcome is that the original single face object is *replaced* by a new object containing two faces.
        // Therefore, the final number of objects in the scene should still be 1, but this object should have 2 faces.
        std::cout << "\n--- FINAL VERIFICATION ---" << std::endl;
        
        size_t finalObjectCount = objects_.size();
        bool testPassed = false;
        if (finalObjectCount == 1) {
            SceneObject* finalObj = get_all_objects()[0];
            TopExp_Explorer faceExp(*finalObj->get_shape(), TopAbs_FACE);
            int faceCount = 0;
            for (; faceExp.More(); faceExp.Next()) faceCount++;
            if (faceCount == 2) {
                testPassed = true;
            } else {
                 std::cout << "TEST FAILED: Final object has " << faceCount << " faces, but expected 2." << std::endl;
            }
        } else {
             std::cout << "TEST FAILED: Expected 1 final object, but found " << finalObjectCount << "." << std::endl;
        }

        if (testPassed) {
            std::cout << "TEST PASSED: Final object count is 1, and it contains 2 faces, as expected." << std::endl;
        } else {
            std::cout << "This likely indicates a failure in the parametric splitting logic or loop detection." << std::endl;
        }

        std::cout << "\n--- Final Object Analysis ---" << std::endl;
        for (const auto& [id, obj_ptr] : objects_) {
            AnalyzeShape(*obj_ptr->get_shape(), "Final Object " + std::to_string(id));
        }
        
        std::cout << "\n--- FACE SPLITTING TEST FINISHED ---\n" << std::endl;
    }

} // namespace Urbaxio