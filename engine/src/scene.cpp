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
#include <GeomAbs_SurfaceType.hxx> // For checking plane type
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
#include <BRepLProp_SLProps.hxx> // For getting face properties like normal
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Cut.hxx> // For subtraction
#include <BRepAlgoAPI_Splitter.hxx> // For splitting faces
#include <BRepClass_FaceClassifier.hxx> // For checking if a point is on a face
#include <BRepCheck_Analyzer.hxx>
#include <ShapeFix_Shape.hxx>
#include <TopTools_ListOfShape.hxx> // <-- REQUIRED FOR SetArguments/SetTools
#include <Standard_Failure.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx> // For vertex-edge map
#include <TopExp.hxx> // For TopExp::MapShapes
#include <ShapeUpgrade_UnifySameDomain.hxx> // <-- THE MAGIC TOOL
#include <Geom2d_Line.hxx> // <-- FIX: Added missing include
#include <ShapeAnalysis_Surface.hxx> // <-- FIX: Added missing include
#include <BOPAlgo_Splitter.hxx>
#include <BOPAlgo_Alerts.hxx> // For DumpErrors

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

    Scene::Scene() {}
    Scene::~Scene() = default;

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
    // --- NEW ARCHITECTURE ---
    // 1. Proactively split existing lines at the new line's endpoints.
    // This ensures the line graph is connected *before* we do anything else.
    // 2. Find T-junction intersections between the new line segment and existing lines.
    // 3. Add the new line, now potentially split by the T-junctions.
    // 4. Add the new segments and check for new faces for each one.

    void Scene::AddUserLine(const glm::vec3& start, const glm::vec3& end) {
        // 1. Proactively split existing lines at the new line's endpoints.
        // This ensures the line graph is connected *before* we do anything else.
        
        // Collect lines to split to avoid iterator invalidation
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

        // Split all existing lines that were T-intersected
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
        uint64_t id1 = AddSingleLineSegment(originalLine.start, canonicalSplitPoint);
        uint64_t id2 = AddSingleLineSegment(canonicalSplitPoint, originalLine.end);
        
        // 3. Now, explicitly try to find faces for the new segments
        // We only need to check one of the new segments, as they are connected
        if(id1 != 0) FindAndCreateFaces(id1);
        
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
            return; // No closed loop found
        }
        
        // --- A closed loop was found ---
        std::vector<glm::vec3> finalOrderedVertices;
        finalOrderedVertices.push_back(pA);
        finalOrderedVertices.push_back(pB);
        if (!pathVerticesCollector.empty()) {
            for (size_t i = 0; i < pathVerticesCollector.size() - 1; ++i) {
                finalOrderedVertices.push_back(pathVerticesCollector[i]);
            }
        }
        if (finalOrderedVertices.size() < 3) return;

        // --- NEW LOGIC: Prioritize splitting existing faces ---
        for (const auto& [id, obj_ptr] : objects_) {
            SceneObject* hostObject = obj_ptr.get();
            if (!hostObject || !hostObject->has_shape()) continue;

            TopExp_Explorer faceExplorer(*hostObject->get_shape(), TopAbs_FACE);
            for (; faceExplorer.More(); faceExplorer.Next()) {
                const TopoDS_Face& candidateFace = TopoDS::Face(faceExplorer.Current());
                
                BRepClass_FaceClassifier classifier;
                bool allPointsOnFace = true;
                for (const auto& vertex : finalOrderedVertices) {
                    classifier.Perform(candidateFace, gp_Pnt(vertex.x, vertex.y, vertex.z), SCENE_POINT_EQUALITY_TOLERANCE);
                    if (classifier.State() != TopAbs_ON && classifier.State() != TopAbs_IN) {
                        allPointsOnFace = false;
                        break;
                    }
                }

                if (allPointsOnFace) {
                    std::cout << "Scene: Detected new loop on existing face of object " << id << ". Attempting to split." << std::endl;
                    BRepBuilderAPI_MakeWire wireMaker;
                    for (size_t i = 0; i < finalOrderedVertices.size(); ++i) {
                        const auto& p1 = finalOrderedVertices[i];
                        const auto& p2 = finalOrderedVertices[(i + 1) % finalOrderedVertices.size()];
                        if (!AreVec3Equal(p1, p2)) {
                             wireMaker.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(p1.x, p1.y, p1.z), gp_Pnt(p2.x, p2.y, p2.z)));
                        }
                    }
                    if (!wireMaker.IsDone() || wireMaker.Wire().IsNull()) {
                         std::cerr << "Scene Split Error: Failed to create wire from new loop." << std::endl;
                         return; // Abort this face-finding attempt
                    }
                    
                    TopoDS_Wire cuttingWire = wireMaker.Wire();
                    
                    // --- CORRECTED USAGE OF BRepAlgoAPI_Splitter ---
                    BRepAlgoAPI_Splitter splitter;
                    TopTools_ListOfShape arguments;
                    arguments.Append(*hostObject->get_shape());
                    TopTools_ListOfShape tools;
                    tools.Append(cuttingWire);
                    splitter.SetArguments(arguments);
                    splitter.SetTools(tools);
                    splitter.Build();
                    
                    if (splitter.IsDone()) {
                        hostObject->set_shape(Urbaxio::CadKernel::OCCT_ShapeUniquePtr(new TopoDS_Shape(splitter.Shape())));
                        UpdateObjectBoundary(hostObject);
                        hostObject->set_mesh_buffers(Urbaxio::CadKernel::TriangulateShape(*hostObject->get_shape()));
                        hostObject->vao = 0; // Mark for re-upload to GPU
                        std::cout << "Scene: Object " << id << " was successfully split and updated." << std::endl;
                    } else {
                        std::cerr << "Scene Split Error: BRepAlgoAPI_Splitter failed." << std::endl;
                    }
                    // IMPORTANT: We found our host face and attempted a split. We must exit now.
                    return; 
                }
            }
        }
        
        // --- If we reach here, no host face was found. Create a new face in space. ---
        gp_Pln cyclePlane;
        if (ArePointsCoplanar(finalOrderedVertices, cyclePlane)) {
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
                // Mark these lines as 'used' so they cannot form other faces
                lines_[line_id].usedInFace = true;
                new_boundary_line_ids.insert(line_id);
                // Remove from the set of old lines, so we know it's still needed
                if(old_line_ids_to_check.count(line_id)) {
                    old_line_ids_to_check.erase(line_id);
                }
            }
        }

        // Any line ID left in old_line_ids_to_check is no longer part of the boundary and should be removed.
        // This handles cases where faces merge and internal edges are removed.
        for (uint64_t dead_line_id : old_line_ids_to_check) {
            RemoveLine(dead_line_id);
        }
        
        // Update the object's list of boundary lines
        obj->boundaryLineIDs = new_boundary_line_ids;
    }
    


    // --- PUSH/PULL (MODIFIED) ---
    TopoDS_Face Scene::FindOriginalFace(const TopoDS_Shape& shape, const std::vector<glm::vec3>& faceVertices, const glm::vec3& faceNormal) {
        if (faceVertices.empty()) return TopoDS_Face();
        
        std::cout << "DEBUG: FindOriginalFace - Shape type: " << shape.ShapeType() << ", Target normal: (" << faceNormal.x << ", " << faceNormal.y << ", " << faceNormal.z << ")" << std::endl;
        
        // Calculate center of selected face vertices for proximity scoring
        glm::vec3 selectedFaceCenter(0.0f);
        for (const auto& v : faceVertices) {
            selectedFaceCenter += v;
        }
        selectedFaceCenter /= float(faceVertices.size());
        std::cout << "DEBUG: Selected face center: (" << selectedFaceCenter.x << ", " << selectedFaceCenter.y << ", " << selectedFaceCenter.z << ")" << std::endl;
        
        gp_Dir targetNormal(faceNormal.x, faceNormal.y, faceNormal.z);
        
        struct FaceCandidate {
            TopoDS_Face face;
            float score;
            int matchingVertices;
            bool normalMatches;
            float centerDistance;
        };
        
        std::vector<FaceCandidate> candidates;

        TopExp_Explorer faceExplorer(shape, TopAbs_FACE);
        int face_count = 0;
        for (; faceExplorer.More(); faceExplorer.Next()) {
            face_count++;
            TopoDS_Face candidateFace = TopoDS::Face(faceExplorer.Current());
            
            try {
                BRepAdaptor_Surface surfaceAdaptor(candidateFace, Standard_False);
                
                // 1. Check Normal
                gp_Pln plane = surfaceAdaptor.Plane();
                gp_Dir occtNormal = plane.Axis().Direction();
                if (candidateFace.Orientation() == TopAbs_REVERSED) {
                    occtNormal.Reverse();
                }
                
                // Tighter tolerance for normal matching
                bool normalMatches = occtNormal.IsParallel(targetNormal, 0.1); // Was 0.3
                
                // 2. Count matching vertices
                TopExp_Explorer vertexExplorer(candidateFace, TopAbs_VERTEX);
                int matchingVertices = 0;
                glm::vec3 faceCenterSum(0.0f);
                int vertexCount = 0;
                
                while (vertexExplorer.More()) {
                    gp_Pnt occt_p = BRep_Tool::Pnt(TopoDS::Vertex(vertexExplorer.Current()));
                    glm::vec3 candidateVertex(occt_p.X(), occt_p.Y(), occt_p.Z());
                    faceCenterSum += candidateVertex;
                    vertexCount++;
                    
                    for(const auto& v : faceVertices) {
                        if (occt_p.IsEqual(gp_Pnt(v.x, v.y, v.z), SCENE_POINT_EQUALITY_TOLERANCE)) {
                            matchingVertices++;
                            break;
                        }
                    }
                    vertexExplorer.Next();
                }
                
                // 3. Calculate face center and distance to selected center
                glm::vec3 candidateFaceCenter = (vertexCount > 0) ? faceCenterSum / float(vertexCount) : glm::vec3(0.0f);
                float centerDistance = glm::distance(selectedFaceCenter, candidateFaceCenter);
                
                // 4. Calculate composite score
                float score = 0.0f;
                if (matchingVertices > 0) {
                    score += matchingVertices * 100.0f; // High weight for vertex matches
                    if (normalMatches) score += 50.0f; // Bonus for normal match
                    score -= centerDistance; // Penalty for distance from selected center
                }
                
                std::cout << "DEBUG: Face " << face_count << " normal: (" << occtNormal.X() << ", " << occtNormal.Y() << ", " << occtNormal.Z() 
                         << "), vertices: " << matchingVertices << "/" << faceVertices.size() 
                         << ", center dist: " << centerDistance << ", score: " << score << ", normal match: " << (normalMatches ? "YES":"NO") << std::endl;
                
                if (matchingVertices > 0) {
                    candidates.push_back({candidateFace, score, matchingVertices, normalMatches, centerDistance});
                }
                
            } catch (const Standard_Failure& e) {
                std::cout << "DEBUG: Failed to process face " << face_count << ": " << e.GetMessageString() << std::endl;
                continue;
            }
        }
        
        std::cout << "DEBUG: Total faces found: " << face_count << ", Candidates: " << candidates.size() << std::endl;
        
        if (candidates.empty()) {
            std::cout << "DEBUG: No matching faces found!" << std::endl;
            return TopoDS_Face();
        }
        
        // Sort by score (highest first)
        std::sort(candidates.begin(), candidates.end(), [](const FaceCandidate& a, const FaceCandidate& b) {
            return a.score > b.score;
        });
        
        const auto& best = candidates[0];
        std::cout << "DEBUG: Selected best face with score: " << best.score 
                  << ", matching vertices: " << best.matchingVertices 
                  << ", normal match: " << best.normalMatches << std::endl;
        
        return best.face;
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
            gp_Vec extrudeVector(direction.x * distance, direction.y * distance, direction.z * distance);
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

        std::cout << "Scene: Cleared all objects and lines from engine state." << std::endl;
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
        // The expected outcome is that the original face is split into two new faces:
        // 1. The inner rectangle defined by the 'П' and the edge.
        // 2. The outer C-shaped face.
        // Therefore, the final number of objects in the scene should be 2.
        std::cout << "\n--- FINAL VERIFICATION ---" << std::endl;
        
        size_t finalObjectCount = objects_.size();
        bool testPassed = (finalObjectCount == 2);

        if (testPassed) {
            std::cout << "TEST PASSED: Final object count is 2, as expected." << std::endl;
        } else {
            std::cout << "TEST FAILED: Expected 2 final objects, but found " << finalObjectCount << "." << std::endl;
            std::cout << "This likely indicates a failure in one of the following steps:" << std::endl;
            std::cout << "  a) FindAndCreateFaces failed to detect the closed loop across multiple lines." << std::endl;
            std::cout << "  b) The final splitting operation with the found loop failed." << std::endl;
            std::cout << "  c) The loop detection logic needs improvement for complex scenarios." << std::endl;
        }

        std::cout << "\n--- Final Object Analysis ---" << std::endl;
        for (const auto& [id, obj_ptr] : objects_) {
            AnalyzeShape(*obj_ptr->get_shape(), "Final Object " + std::to_string(id));
        }
        
        std::cout << "\n--- FACE SPLITTING TEST FINISHED ---\n" << std::endl;
    }

} // namespace Urbaxio