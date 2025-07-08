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
#include <TopoDS.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>
#include <TopExp_Explorer.hxx>
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
#include <BRepCheck_Analyzer.hxx>
#include <ShapeFix_Shape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <Standard_Failure.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>

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

    // Public method to add a line, which now handles intersections.
    void Scene::AddUserLine(const glm::vec3& start, const glm::vec3& end) {
        std::map<uint64_t, glm::vec3> intersections_on_existing_lines;
        std::vector<glm::vec3> split_points_on_new_line;

        // Find all intersections with existing lines
        for (const auto& [existing_id, existing_line] : lines_) {
            glm::vec3 intersection_point;
            if (LineSegmentIntersection(start, end, existing_line.start, existing_line.end, intersection_point)) {
                intersections_on_existing_lines[existing_id] = intersection_point;
                split_points_on_new_line.push_back(intersection_point);
            }
        }

        // Split all existing lines that were intersected
        for (const auto& [line_id, point] : intersections_on_existing_lines) {
            SplitLineAtPoint(line_id, point);
        }
        
        // Add the new line, potentially splitting it as well
        std::vector<glm::vec3> all_points = { start };
        all_points.insert(all_points.end(), split_points_on_new_line.begin(), split_points_on_new_line.end());
        all_points.push_back(end);

        // Sort all points along the new line's direction
        glm::vec3 dir = end - start;
        std::sort(all_points.begin(), all_points.end(), 
            [&start, &dir](const glm::vec3& a, const glm::vec3& b) {
                if (glm::length2(dir) < 1e-9) return false;
                return glm::dot(a - start, dir) < glm::dot(b - start, dir);
            });
            
        // Remove duplicate points
        all_points.erase(std::unique(all_points.begin(), all_points.end(), AreVec3Equal), all_points.end());

        // Add the new line as segments
        for (size_t i = 0; i < all_points.size() - 1; ++i) {
            AddSingleLineSegment(all_points[i], all_points[i+1]);
        }
    }

    // This is the internal, "dumb" version of AddUserLine. It just adds a segment.
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
                return 0; // Duplicate line
            }
        }
        
        uint64_t newLineId = next_line_id_++;
        lines_[newLineId] = {canonicalStart, canonicalEnd, false};

        vertexAdjacency_[canonicalStart].push_back(newLineId);
        vertexAdjacency_[canonicalEnd].push_back(newLineId);
        
        FindAndCreateFaces(newLineId);
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

        // --- NEW: More robust coplanarity check ---
        // Create a plane with 3 points and check if the 4th is on it.
        glm::vec3 plane_normal = glm::cross(d1, p3 - p1);
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

        // Lines are coplanar, now find intersection point
        glm::vec3 d1_cross_d2 = glm::cross(d1, d2);
        float d1_cross_d2_lenSq = glm::length2(d1_cross_d2);

        // Check if lines are parallel
        if (d1_cross_d2_lenSq < EPSILON * EPSILON) {
            return false; // Parallel lines, we don't handle overlapping case for now.
        }
        
        glm::vec3 p3_p1_cross_d2 = glm::cross(p3 - p1, d2);

        float t = glm::dot(p3_p1_cross_d2, d1_cross_d2) / d1_cross_d2_lenSq;
        
        // --- NEW: Stricter bounds check to avoid intersections at endpoints ---
        if (t < EPSILON || t > 1.0f - EPSILON) {
            return false;
        }

        float u = glm::dot(glm::cross(p3 - p1, d1), d1_cross_d2) / d1_cross_d2_lenSq;
        
        // --- NEW: Stricter bounds check to avoid intersections at endpoints ---
        if (u < EPSILON || u > 1.0f - EPSILON) {
            return false;
        }

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

            if (lineId == originatingLineId || line_it->second.usedInFace || visitedLinesDFS.count(lineId)) {
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
                    Urbaxio::CadKernel::MeshBuffers mesh_data = Urbaxio::CadKernel::TriangulateShape(face);
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
        if (line_it == lines_.end() || line_it->second.usedInFace) return;
        
        const Line& newLine = line_it->second;
        const glm::vec3& pA = newLine.start;
        const glm::vec3& pB = newLine.end;
        
        std::vector<glm::vec3> pathVerticesCollector;
        std::vector<uint64_t> pathLineIDsCollector;
        std::set<uint64_t> visitedLinesInCurrentDFS;
        visitedLinesInCurrentDFS.insert(newLineId);
        int recursionDepth = 0;

        if (PerformDFS(pA, pB, pA, pathVerticesCollector, pathLineIDsCollector, visitedLinesInCurrentDFS, newLineId, recursionDepth)) {
            std::vector<glm::vec3> finalOrderedVertices;
            finalOrderedVertices.push_back(pA);
            finalOrderedVertices.push_back(pB);
            if (!pathVerticesCollector.empty()) {
                // The last vertex in the path collector is the same as pA, so we skip it.
                for (size_t i = 0; i < pathVerticesCollector.size() - 1; ++i) {
                    finalOrderedVertices.push_back(pathVerticesCollector[i]);
                }
            } else { return; }
            if (finalOrderedVertices.size() < 3) return;

            gp_Pln cyclePlane;
            if (ArePointsCoplanar(finalOrderedVertices, cyclePlane)) {
                CreateOCCTFace(finalOrderedVertices, cyclePlane);
                
                // Mark all lines in the loop as used
                lines_.at(newLineId).usedInFace = true;
                for (uint64_t lineId : pathLineIDsCollector) {
                    lines_.at(lineId).usedInFace = true;
                }
            }
        }
    }

    // --- Push/Pull Implementation ---

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

    bool Scene::ExtrudeFace(uint64_t objectId, const std::vector<size_t>& faceTriangleIndices, const glm::vec3& direction, float distance) {
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
            
            TopoDS_Shape newFinalShape;
            
            // --- ROBUST BOOLEAN OPERATION LOGIC ---
            // Determine the correct operation based on the extrusion direction relative to the solid's face
            BRepAdaptor_Surface surfaceAdaptor(faceToExtrude, Standard_True); // Use Standard_True for natural UV mapping
            gp_Pln plane = surfaceAdaptor.Plane();
            gp_Dir occtFaceNormal = plane.Axis().Direction();
            if (faceToExtrude.Orientation() == TopAbs_REVERSED) {
                occtFaceNormal.Reverse();
            }

            double dotProduct = extrudeVector.Dot(gp_Vec(occtFaceNormal));
            std::cout << "Scene: Push/Pull - Face normal dot extrusion vector = " << dotProduct << std::endl;

            if (originalShape->ShapeType() == TopAbs_FACE) {
                // A standalone face is always extruded into a solid, the boolean operation is irrelevant.
                newFinalShape = prismShape;
                std::cout << "Scene: Push/Pull - Standalone face converted to solid." << std::endl;

            } else if (dotProduct >= -1e-6) { // Use tolerance for the parallel (dot=0) case
                // Extruding outwards from the solid, or parallel to the face -> Add material
                std::cout << "Scene: Push/Pull - Detected outward extrusion. Using Fuse operation." << std::endl;
                BRepAlgoAPI_Fuse fuseAlgo(*originalShape, prismShape);
                if (!fuseAlgo.IsDone()) {
                    std::cerr << "OCCT Error: BRepAlgoAPI_Fuse failed." << std::endl;
                    AnalyzeShape(fuseAlgo.Shape(), "FUSE FAILED SHAPE");
                    return false;
                }
                newFinalShape = fuseAlgo.Shape();
                if (fuseAlgo.HasErrors()) { std::cerr << "OCCT Warning: Fuse operation has errors." << std::endl; }
                if (fuseAlgo.HasWarnings()) { std::cout << "OCCT Info: Fuse operation has warnings." << std::endl; }

            } else { // dotProduct < 0
                // Extruding inwards into the solid -> Remove material
                std::cout << "Scene: Push/Pull - Detected inward extrusion. Using Cut operation." << std::endl;
                BRepAlgoAPI_Cut cutAlgo(*originalShape, prismShape);
                if (!cutAlgo.IsDone()) {
                    std::cerr << "OCCT Error: BRepAlgoAPI_Cut failed." << std::endl;
                    AnalyzeShape(cutAlgo.Shape(), "CUT FAILED SHAPE");
                    return false;
                }
                newFinalShape = cutAlgo.Shape();
                if (cutAlgo.HasErrors()) { std::cerr << "OCCT Warning: Cut operation has errors." << std::endl; }
                if (cutAlgo.HasWarnings()) { std::cout << "OCCT Info: Cut operation has warnings." << std::endl; }
            }
            
            // --- VALIDATION AND UPDATE ---
            AnalyzeShape(newFinalShape, "FINAL RESULT (Before Healing)");
            
            if (newFinalShape.IsNull()) {
                std::cerr << "OCCT Error: Resulting shape is null after boolean operation!" << std::endl;
                return false;
            }
            
            BRepCheck_Analyzer analyzer(newFinalShape);
            if (!analyzer.IsValid()) {
                std::cerr << "OCCT Warning: Result shape is not valid. Attempting to heal..." << std::endl;
                try {
                    ShapeFix_Shape shapeFixer(newFinalShape);
                    shapeFixer.Perform();
                    TopoDS_Shape healedShape = shapeFixer.Shape();
                    
                    BRepCheck_Analyzer healedAnalyzer(healedShape);
                    if (healedAnalyzer.IsValid()) {
                        std::cout << "DEBUG: Shape healing successful." << std::endl;
                        newFinalShape = healedShape;
                        AnalyzeShape(newFinalShape, "FINAL RESULT (After Healing)");
                    } else {
                        std::cout << "DEBUG: Shape healing failed, using original (invalid) result." << std::endl;
                    }
                } catch (const Standard_Failure& e) {
                    std::cout << "DEBUG: Shape healing threw exception: " << e.GetMessageString() << std::endl;
                }
            } else {
                std::cout << "DEBUG: Result shape is valid." << std::endl;
            }
            
            obj->set_shape(Urbaxio::CadKernel::OCCT_ShapeUniquePtr(new TopoDS_Shape(newFinalShape)));
            obj->set_mesh_buffers(Urbaxio::CadKernel::TriangulateShape(newFinalShape));
            obj->vao = 0; // Mark for re-upload to GPU
            
            std::cout << "Scene: Push/Pull successful. Object " << objectId << " updated." << std::endl;

        } catch (const Standard_Failure& e) {
            std::cerr << "OCCT Exception during ExtrudeFace: " << e.GetMessageString() << std::endl;
            return false;
        }
        return true;
    }


} // namespace Urbaxio