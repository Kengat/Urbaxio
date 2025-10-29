// engine/src/scene.cpp
#define GLM_ENABLE_EXPERIMENTAL
#include "engine/scene.h"
#include "engine/scene_object.h"
#include <cad_kernel/cad_kernel.h>
#include <cad_kernel/MeshBuffers.h> // Required for TriangulateShape return type

#include <utility>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <set>
#include <cmath>
#include <algorithm> // for std::reverse, std::remove
#include <deque> // For GrowCoplanarRegion helper

#include <glm/gtx/norm.hpp>
#include <glm/common.hpp> // For epsilonEqual
#include <glm/gtx/intersect.hpp> // For glm::intersectRayPlane

// OCCT Includes
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Trsf.hxx>
#include <Geom_Surface.hxx>
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
#include <BRepBuilderAPI_Transform.hxx>
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
#include <TopExp.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopTools_HSequenceOfShape.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <ShapeBuild_ReShape.hxx>
#include <TColStd_MapOfAsciiString.hxx>
#include <Geom2d_Line.hxx>
#include <ShapeAnalysis_Surface.hxx>
#include <BOPAlgo_Splitter.hxx>
#include <BOPAlgo_Alerts.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Dir2d.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx> // For GrowCoplanarRegion helper
#include <TopTools_IndexedMapOfShape.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx> 
#include <ShapeFix_Wire.hxx>
#include <ShapeExtend_Status.hxx>
#include <sstream>
#include <BinTools.hxx>

// --- NEW: Additional includes for deformation algorithm ---
#include <BRepTools_ReShape.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Compound.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs.hxx>
#include <BRep_Builder.hxx>
#include <BRepLib.hxx> // ADD THIS LINE for BuildCurves3d and SameParameter
#include <BRepAdaptor_Surface.hxx>
#include <BRepLProp_SLProps.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>

// --- NEW: Additional includes for detailed validity check ---
#include <BRepCheck_ListIteratorOfListOfStatus.hxx>
#include <map>
#include <BRepCheck_Status.hxx> // <-- THE CRUCIAL INCLUDE THAT WAS MISSING
#include <ShapeFix_Face.hxx>

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

    // --- NEW: Helpers for coplanar region detection and boundary extraction ---
    static bool IsPlanar(const TopoDS_Face& f, gp_Pln& out) {
        BRepAdaptor_Surface sa(f, Standard_True);
        if (sa.GetType() != GeomAbs_Plane) return false;
        out = sa.Plane();
        return true;
    }

    // "equality" of planes with tolerances
    static bool SamePlane(const gp_Pln& a, const gp_Pln& b,
                          double angTol = 1.0e-6, double linTol = 1.0e-6)
    {
        if (!a.Axis().Direction().IsParallel(b.Axis().Direction(), angTol)) return false;
        // take a point from plane b and measure the distance to plane a
        return std::abs(a.Distance(b.Location())) <= linTol;
    }

    // gather a connected coplanar region around a seed
    static void GrowCoplanarRegion(const TopoDS_Shape& body, const TopoDS_Face& seed,
                                   TopTools_ListOfShape& regionFaces, gp_Pln& regionPlane)
    {
        if (!IsPlanar(seed, regionPlane)) return;

        TopTools_IndexedDataMapOfShapeListOfShape edge2faces;
        TopExp::MapShapesAndAncestors(body, TopAbs_EDGE, TopAbs_FACE, edge2faces);

        std::unordered_set<TopoDS_Shape, TopTools_ShapeMapHasher> visited;
        std::deque<TopoDS_Face> q;
        q.push_back(seed); visited.insert(seed);

        while (!q.empty())
        {
            TopoDS_Face f = q.front(); q.pop_front();
            regionFaces.Append(f);

            // neighbors by edges
            for (TopExp_Explorer ex(f, TopAbs_EDGE); ex.More(); ex.Next())
            {
                const TopoDS_Shape& e = ex.Current();
                const TopTools_ListOfShape& parents = edge2faces.FindFromKey(e);
                for (TopTools_ListIteratorOfListOfShape it(parents); it.More(); it.Next())
                {
                    TopoDS_Face n = TopoDS::Face(it.Value());
                    if (n.IsSame(f) || visited.count(n)) continue;

                    gp_Pln np; if (!IsPlanar(n, np)) continue;
                    if (!SamePlane(regionPlane, np))  continue;

                    visited.insert(n);
                    q.push_back(n);
                }
            }
        }
    }

    // outer wire of the region (edges that have only one owner within the region)
    static TopoDS_Wire OuterWireOfRegion(const TopTools_ListOfShape& regionFaces)
    {
        std::unordered_map<TopoDS_Shape,int,TopTools_ShapeMapHasher> counter;

        for (TopTools_ListIteratorOfListOfShape it(regionFaces); it.More(); it.Next())
        {
            TopoDS_Face f = TopoDS::Face(it.Value());
            for (TopExp_Explorer ex(f, TopAbs_EDGE); ex.More(); ex.Next())
            {
                const TopoDS_Shape& e = ex.Current();
                ++counter[e];
            }
        }

        BRepBuilderAPI_MakeWire mw;
        for (const auto& kv : counter)
        {
            if (kv.second == 1) mw.Add(TopoDS::Edge(kv.first)); // boundary
        }

        TopoDS_Wire raw = mw.Wire();
        Handle(ShapeFix_Wire) fix = new ShapeFix_Wire();
        fix->Load(raw);
        fix->Perform();
        return fix->Wire();
    }

    static TopoDS_Shape RefineFacesWithShapeFix(const TopoDS_Shape& shape, double precision, double maxTol)
    {
        Handle(ShapeBuild_ReShape) reshaper = new ShapeBuild_ReShape();
        bool modified = false;

        for (TopExp_Explorer faceExp(shape, TopAbs_FACE); faceExp.More(); faceExp.Next())
        {
            TopoDS_Face face = TopoDS::Face(faceExp.Current());

            ShapeFix_Face fixer;
            fixer.Init(face);
            fixer.SetPrecision(precision);
            fixer.SetMinTolerance(precision * 0.25);
            fixer.SetMaxTolerance(maxTol);

            Handle(ShapeFix_Wire) wireFix = fixer.FixWireTool();
            if (!wireFix.IsNull()) {
                wireFix->SetPrecision(precision);
                wireFix->SetMaxTolerance(maxTol);
                wireFix->SetMinTolerance(precision * 0.25);
            }

            fixer.Perform();

            TopoDS_Face fixedFace = fixer.Face();
            if (!fixedFace.IsNull() && !fixedFace.IsSame(face)) {
                reshaper->Replace(face, fixedFace);
                modified = true;
            }
        }

        if (modified) {
            return reshaper->Apply(shape);
        }
        return shape;
    }

    // --- START OF MODIFIED HELPER FUNCTION ---
    // Corrected for OCCT's plain enum `BRepCheck_Status` with the exact member list
    static std::string BRepCheckStatusToString(const BRepCheck_Status theStatus)
    {
        switch (theStatus)
        {
            case BRepCheck_NoError: return "NoError";
            case BRepCheck_InvalidPointOnCurve: return "InvalidPointOnCurve";
            case BRepCheck_InvalidPointOnCurveOnSurface: return "InvalidPointOnCurveOnSurface";
            case BRepCheck_InvalidPointOnSurface: return "InvalidPointOnSurface";
            case BRepCheck_No3DCurve: return "No3DCurve";
            case BRepCheck_Multiple3DCurve: return "Multiple3DCurve";
            case BRepCheck_Invalid3DCurve: return "Invalid3DCurve";
            case BRepCheck_NoCurveOnSurface: return "NoCurveOnSurface";
            case BRepCheck_InvalidCurveOnSurface: return "InvalidCurveOnSurface";
            case BRepCheck_InvalidCurveOnClosedSurface: return "InvalidCurveOnClosedSurface";
            case BRepCheck_InvalidSameRangeFlag: return "InvalidSameRangeFlag";
            case BRepCheck_InvalidSameParameterFlag: return "InvalidSameParameterFlag";
            case BRepCheck_InvalidDegeneratedFlag: return "InvalidDegeneratedFlag";
            case BRepCheck_FreeEdge: return "FreeEdge";
            case BRepCheck_InvalidMultiConnexity: return "InvalidMultiConnexity";
            case BRepCheck_InvalidRange: return "InvalidRange";
            case BRepCheck_EmptyWire: return "EmptyWire";
            case BRepCheck_RedundantEdge: return "RedundantEdge";
            case BRepCheck_SelfIntersectingWire: return "SelfIntersectingWire";
            case BRepCheck_NoSurface: return "NoSurface";
            case BRepCheck_InvalidWire: return "InvalidWire";
            case BRepCheck_RedundantWire: return "RedundantWire";
            case BRepCheck_IntersectingWires: return "IntersectingWires";
            case BRepCheck_InvalidImbricationOfWires: return "InvalidImbricationOfWires";
            case BRepCheck_EmptyShell: return "EmptyShell";
            case BRepCheck_RedundantFace: return "RedundantFace";
            case BRepCheck_InvalidImbricationOfShells: return "InvalidImbricationOfShells";
            case BRepCheck_UnorientableShape: return "UnorientableShape";
            case BRepCheck_NotClosed: return "NotClosed";
            case BRepCheck_NotConnected: return "NotConnected";
            case BRepCheck_SubshapeNotInShape: return "SubshapeNotInShape";
            case BRepCheck_BadOrientation: return "BadOrientation";
            case BRepCheck_BadOrientationOfSubshape: return "BadOrientationOfSubshape";
            case BRepCheck_InvalidPolygonOnTriangulation: return "InvalidPolygonOnTriangulation";
            case BRepCheck_InvalidToleranceValue: return "InvalidToleranceValue";
            case BRepCheck_EnclosedRegion: return "EnclosedRegion";
            case BRepCheck_CheckFail: return "CheckFail";
            default: return "UnknownError";
        }
    }
    // --- END OF MODIFIED HELPER FUNCTION ---

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

    // --- NEW: Scene Memento Implementation ---
    std::unique_ptr<SceneState> Scene::CaptureState() {
        auto state = std::make_unique<SceneState>();
        
        state->lines = this->lines_;
        state->vertexAdjacency = this->vertexAdjacency_;
        state->nextObjectId = this->next_object_id_;
        state->nextLineId = this->next_line_id_;
        
        for (const auto& [id, obj_ptr] : this->objects_) {
            ObjectState objState;
            objState.id = id;
            objState.name = obj_ptr->get_name();
            if (obj_ptr->has_shape()) {
                const TopoDS_Shape* shape = obj_ptr->get_shape();
                if (shape && !shape->IsNull()) {
                    std::stringstream ss;
                    try {
                        BinTools::Write(*shape, ss);
                        std::string const& s = ss.str();
                        objState.serializedShape = std::vector<char>(s.begin(), s.end());
                    } catch (...) { /* error */ }
                }
            }
            state->objects[id] = objState;
        }
        return state;
    }
    
    void Scene::RestoreState(const SceneState& state) {
        this->lines_ = state.lines;
        this->vertexAdjacency_ = state.vertexAdjacency;
        this->next_object_id_ = state.nextObjectId;
        this->next_line_id_ = state.nextLineId;

        std::set<uint64_t> newStateObjectIds;
        for (const auto& [id, objState] : state.objects) {
            newStateObjectIds.insert(id);
        }

        for (auto it = this->objects_.begin(); it != this->objects_.end(); ) {
            if (newStateObjectIds.find(it->first) == newStateObjectIds.end()) {
                it = this->objects_.erase(it);
            } else { ++it; }
        }
        
        for (const auto& [id, objState] : state.objects) {
            SceneObject* obj = this->get_object_by_id(id);
            if (!obj) {
                obj = this->create_object(objState.name);
                // Note: create_object assigns a new ID, but for undo/redo we want to preserve the original ID.
                // This would require a custom object creation for full fidelity, but for now we ensure the object exists.
            }

            if (!objState.serializedShape.empty()) {
                TopoDS_Shape restoredShape;
                std::stringstream ss(std::string(objState.serializedShape.begin(), objState.serializedShape.end()));
                try {
                    BinTools::Read(restoredShape, ss);
                    obj->set_shape(Urbaxio::CadKernel::OCCT_ShapeUniquePtr(new TopoDS_Shape(restoredShape)));
                    obj->set_mesh_buffers(Urbaxio::CadKernel::TriangulateShape(*obj->get_shape()));
                    obj->vao = 0;
                    this->UpdateObjectBoundary(obj);
                } catch(...) { /* error */ }
            } else if (obj->has_shape()) {
                obj->set_shape(nullptr);
                obj->set_mesh_buffers({});
                obj->vao = 0;
            }
        }
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
        if (vertexAdjacency_.count(line.start)) {
            auto& startAdj = vertexAdjacency_.at(line.start);
            startAdj.erase(std::remove(startAdj.begin(), startAdj.end(), lineId), startAdj.end());
            if (startAdj.empty()) {
                vertexAdjacency_.erase(line.start);
            }
        }

        // Remove from end vertex adjacency
        if (vertexAdjacency_.count(line.end)) {
            auto& endAdj = vertexAdjacency_.at(line.end);
            endAdj.erase(std::remove(endAdj.begin(), endAdj.end(), lineId), endAdj.end());
            if (endAdj.empty()) {
                vertexAdjacency_.erase(line.end);
            }
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
                    TopoDS_Face foundFace = FindOriginalFace(obj_ptr.get(), *obj_ptr->get_shape(), finalOrderedVertices, loopNormal);
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
                // --- Step 1: Heal the host shape to ensure it's valid for boolean operations ---
                ShapeFix_Shape shapeFixer(*hostObject->get_shape());
                shapeFixer.Perform();
                TopoDS_Shape healedShape = shapeFixer.Shape();
                AnalyzeShape(healedShape, "Healed Host Shape BEFORE split");
                // Re-find the face on the healed shape using the context
                Handle(ShapeBuild_ReShape) context = shapeFixer.Context();
                TopoDS_Shape healedFaceShape = context->Apply(originalHostFace);
                if (healedFaceShape.IsNull() || healedFaceShape.ShapeType() != TopAbs_FACE) {
                     throw Standard_Failure("Could not re-find host face after healing.");
                }
                TopoDS_Face healedHostFace = TopoDS::Face(healedFaceShape);
                AnalyzeShape(healedHostFace, "Healed Host Face BEFORE split");
                Handle(Geom_Surface) hostSurface = BRep_Tool::Surface(healedHostFace);
                if (hostSurface.IsNull()) {
                    throw Standard_Failure("Host face has no underlying surface.");
                }
                Handle(ShapeAnalysis_Surface) analysis = new ShapeAnalysis_Surface(hostSurface);
                // --- Step 2: Create a list of topologically sound edges from the loop vertices ---
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
                    uv1.SetX(std::max(umin, std::min(umax, uv1.X())));
                    uv1.SetY(std::max(vmin, std::min(vmax, uv1.Y())));
                    uv2.SetX(std::max(umin, std::min(umax, uv2.X())));
                    uv2.SetY(std::max(vmin, std::min(vmax, uv2.Y())));
                    if (uv1.IsEqual(uv2, Precision::Confusion())) continue;
                    Handle(Geom2d_Line) pcurve = new Geom2d_Line(uv1, gp_Dir2d(uv2.X() - uv1.X(), uv2.Y() - uv1.Y()));
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
                AnalyzeShape(cuttingWire, "Cutting Wire BEFORE split");
                if (cuttingWire.IsNull() || !BRep_Tool::IsClosed(cuttingWire)) {
                    throw Standard_Failure("The final wire created by ShapeFix_Wire is not a single, closed loop.");
                }
                // --- Step 4: Perform the Split operation ---
                BOPAlgo_Splitter splitter;
                TopTools_ListOfShape arguments;
                arguments.Append(healedShape);
                splitter.SetArguments(arguments);
                TopTools_ListOfShape tools;
                tools.Append(cuttingWire);
                splitter.SetTools(tools);
                splitter.Perform();
                // Count faces before for validation
                TopExp_Explorer faceExpBefore(healedShape, TopAbs_FACE);
                int faceCountBefore = 0;
                for (; faceExpBefore.More(); faceExpBefore.Next()) faceCountBefore++;

                splitter.Perform();

                // ROBUSTNESS: Check for errors from the boolean operation itself.
                if (splitter.HasErrors()) {
                    splitter.GetReport()->Dump(std::cout);
                    throw Standard_Failure("BOPAlgo_Splitter failed to perform operation.");
                }
                
                TopoDS_Shape splitShape = splitter.Shape();

                // --- NEW: Validate that the split actually happened ---
                TopExp_Explorer faceExpAfter(splitShape, TopAbs_FACE);
                int faceCountAfter = 0;
                for (; faceExpAfter.More(); faceExpAfter.Next()) faceCountAfter++;

                if (faceCountAfter <= faceCountBefore) {
                    throw Standard_Failure("BOPAlgo_Splitter completed without errors but failed to split the face. This can indicate a degenerate cutting wire or tolerance issues.");
                }
                AnalyzeShape(splitShape, "Split Shape AFTER BOPAlgo_Splitter");
                ShapeFix_Shape finalFixer(splitShape);
                finalFixer.Perform();
                TopoDS_Shape finalShape = finalFixer.Shape();
                AnalyzeShape(finalShape, "Final Shape AFTER ShapeFix_Shape");
                hostObject->set_shape(Urbaxio::CadKernel::OCCT_ShapeUniquePtr(new TopoDS_Shape(finalShape)));
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
        
        // --- MODIFIED LOGIC ---
        // 1. Remove all old lines associated with this object.
        // This is crucial for rebuilding vertex adjacency correctly.
        for (uint64_t oldLineId : obj->boundaryLineIDs) {
            RemoveLine(oldLineId);
        }
        obj->boundaryLineIDs.clear();
        
        // 2. Extract fresh edges from the shape.
        auto new_edges = ExtractEdgesFromShape(*obj->get_shape());
        
        // 3. Add the new edges as lines and update the object's boundary set.
        for (const auto& edge : new_edges) {
            uint64_t line_id = AddSingleLineSegment(edge.first, edge.second);
            if (line_id != 0) {
                lines_[line_id].usedInFace = true;
                obj->boundaryLineIDs.insert(line_id);
            }
        }
    }
    


    // --- УНИВЕРСАЛЬНЫЙ ГИБРИДНЫЙ ПОДХОД ---
    TopoDS_Face Scene::FindOriginalFace(SceneObject* obj, const TopoDS_Shape& shape, const std::vector<glm::vec3>& faceVertices, const glm::vec3& guideNormal) {
        if (faceVertices.empty() || !obj) return TopoDS_Face();

        // --- ШАГ 1: НАДЕЖНЫЙ ТОПОЛОГИЧЕСКИЙ ПОИСК (для MoveTool) ---
        std::unordered_set<TopoDS_Vertex, TopTools_ShapeMapHasher> targetVertices;
        for (const auto& pos : faceVertices) {
            const TopoDS_Vertex* v = obj->findVertexAtLocation(pos);
            if (v) {
                targetVertices.insert(*v);
            }
        }

        std::vector<TopoDS_Face> candidates;
        if (!targetVertices.empty()) {
            TopExp_Explorer faceExplorer(shape, TopAbs_FACE);
            for (; faceExplorer.More(); faceExplorer.Next()) {
                TopoDS_Face candidateFace = TopoDS::Face(faceExplorer.Current());
                std::unordered_set<TopoDS_Vertex, TopTools_ShapeMapHasher> faceVerticesSet;
                TopExp_Explorer vertexExplorer(candidateFace, TopAbs_VERTEX);
                for (; vertexExplorer.More(); vertexExplorer.Next()) {
                    faceVerticesSet.insert(TopoDS::Vertex(vertexExplorer.Current()));
                }

                bool contains_all_targets = true;
                for (const auto& targetV : targetVertices) {
                    if (faceVerticesSet.find(targetV) == faceVerticesSet.end()) {
                        contains_all_targets = false;
                        break;
                    }
                }
                if (contains_all_targets) {
                    candidates.push_back(candidateFace);
                }
            }
        }
        
        // --- ШАГ 2: ГИБКИЙ ГЕОМЕТРИЧЕСКИЙ ОТКАТ (для LineTool и PushPull) ---
        // Если топологический поиск не дал точного совпадения, используем старый метод, основанный на геометрии.
        if (candidates.empty()) {
            TopExp_Explorer faceExplorer(shape, TopAbs_FACE);
            for (; faceExplorer.More(); faceExplorer.Next()) {
                TopoDS_Face candidateFace = TopoDS::Face(faceExplorer.Current());
                try {
                    BRepAdaptor_Surface surfaceAdaptor(candidateFace, Standard_False);
                    if(surfaceAdaptor.GetType() != GeomAbs_Plane) continue;
                    bool allPointsOnFace = true;
                    for(const auto& v : faceVertices) {
                        BRepClass_FaceClassifier classifier;
                        classifier.Perform(candidateFace, gp_Pnt(v.x, v.y, v.z), 1e-4); 
                        TopAbs_State state = classifier.State();
                        if(state != TopAbs_ON && state != TopAbs_IN) {
                            allPointsOnFace = false;
                            break;
                        }
                    }
                    if (allPointsOnFace) {
                        candidates.push_back(candidateFace);
                    }
                } catch (const Standard_Failure&) {
                    continue;
                }
            }
        }

        // 3. Логика выбора лучшего кандидата (остается без изменений)
        if (candidates.empty()) {
            return TopoDS_Face();
        }
        if (candidates.size() == 1) {
            return candidates[0];
        }
        
        std::vector<TopoDS_Face> alignedCandidates;
        double maxDotAbs = -1.0;
        gp_Dir targetDir(guideNormal.x, guideNormal.y, guideNormal.z);
        for (const auto& candidateFace : candidates) {
            try {
                BRepAdaptor_Surface surfaceAdaptor(candidateFace, Standard_True);
                gp_Pln plane = surfaceAdaptor.Plane();
                gp_Dir candidateDir = plane.Axis().Direction();
                if (candidateFace.Orientation() == TopAbs_REVERSED) candidateDir.Reverse();
                double dot = std::abs(targetDir.Dot(candidateDir));
                if (dot > maxDotAbs) {
                    maxDotAbs = dot;
                }
            } catch (const Standard_Failure&) { continue; }
        }
        const double NORMAL_ALIGNMENT_TOLERANCE = 1e-6;
        if (maxDotAbs > (1.0 - NORMAL_ALIGNMENT_TOLERANCE)) {
            for (const auto& candidateFace : candidates) {
                try {
                    BRepAdaptor_Surface surfaceAdaptor(candidateFace, Standard_True);
                    gp_Pln plane = surfaceAdaptor.Plane();
                    gp_Dir candidateDir = plane.Axis().Direction();
                    if (candidateFace.Orientation() == TopAbs_REVERSED) candidateDir.Reverse();
                    if (std::abs(targetDir.Dot(candidateDir)) >= (maxDotAbs - NORMAL_ALIGNMENT_TOLERANCE)) {
                        alignedCandidates.push_back(candidateFace);
                    }
                } catch (const Standard_Failure&) { continue; }
            }
        }
        if (alignedCandidates.size() == 1) {
            return alignedCandidates[0];
        }
        if (alignedCandidates.empty()) {
            alignedCandidates = candidates;
        }
        
        glm::vec3 targetCenter(0.0f);
        for(const auto& v : faceVertices) targetCenter += v;
        targetCenter /= (float)faceVertices.size();
        TopoDS_Face bestMatch;
        float minDistance = std::numeric_limits<float>::max();
        for (const auto& candidateFace : alignedCandidates) {
            try {
                GProp_GProps props;
                BRepGProp::SurfaceProperties(candidateFace, props);
                gp_Pnt center = props.CentreOfMass();
                float dist = (float)center.Distance(gp_Pnt(targetCenter.x, targetCenter.y, targetCenter.z));
                if (dist < minDistance) {
                    minDistance = dist;
                    bestMatch = candidateFace;
                }
            } catch (const Standard_Failure&) { continue; }
        }
        return bestMatch;
    }
    
    // --- START OF MODIFIED AnalyzeShape FUNCTION ---
    void Scene::AnalyzeShape(const TopoDS_Shape& shape, const std::string& label) {
        if (shape.IsNull()) {
            std::cout << "=== SHAPE ANALYSIS: " << label << " (SHAPE IS NULL) ===" << std::endl;
            return;
        }
        std::cout << "=== SHAPE ANALYSIS: " << label << " ===" << std::endl;

        // Basic info
        std::cout << "Shape Type: " << shape.ShapeType() << " (0=Compound, 1=CompSolid, 2=Solid, 3=Shell, 4=Face, 5=Wire, 6=Edge, 7=Vertex)" << std::endl;

        TopExp_Explorer solidExp(shape, TopAbs_SOLID); int solidCount = 0; for (; solidExp.More(); solidExp.Next()) solidCount++;
        TopExp_Explorer faceExp(shape, TopAbs_FACE); int faceCount = 0; for (; faceExp.More(); faceExp.Next()) faceCount++;
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
                std::cout << "Dimensions: " << (xmax - xmin) << " x " << (ymax - ymin) << " x " << (zmax - zmin) << std::endl;
            } else { std::cout << "Bounding Box: VOID" << std::endl; }
        } catch (const Standard_Failure& e) { std::cout << "Bounding Box: ERROR - " << e.GetMessageString() << std::endl; }

        // Volume/properties
        try {
            GProp_GProps props; BRepGProp::VolumeProperties(shape, props); Standard_Real volume = props.Mass(); gp_Pnt center = props.CentreOfMass();
            std::cout << "Volume: " << volume << std::endl; std::cout << "Center of Mass: (" << center.X() << ", " << center.Y() << ", " << center.Z() << ")" << std::endl;
        } catch (const Standard_Failure& e) { std::cout << "Volume Properties: ERROR - " << e.GetMessageString() << std::endl; }

        // --- Detailed Validity Check ---
        try {
            BRepCheck_Analyzer analyzer(shape);
            if (analyzer.IsValid()) {
                std::cout << "Shape Valid: YES" << std::endl;
            }
            else {
                std::cout << "Shape Valid: NO. Analyzing errors..." << std::endl;
                std::map<std::string, int> errorCounts;
                TopTools_IndexedMapOfShape processedSubShapes;

                for (TopExp_Explorer ex(shape, TopAbs_SHAPE); ex.More(); ex.Next()) {
                    const TopoDS_Shape& subShape = ex.Current();
                    if (processedSubShapes.Contains(subShape)) continue;
                    processedSubShapes.Add(subShape);

                    if (!analyzer.IsValid(subShape)) {
                        Handle(BRepCheck_Result) result = analyzer.Result(subShape);
                        if (!result.IsNull()) {
                            const BRepCheck_ListOfStatus& statuses = result->StatusOnShape(subShape);
                            for (BRepCheck_ListIteratorOfListOfStatus it(statuses); it.More(); it.Next()) {
                                errorCounts[BRepCheckStatusToString(it.Value())]++;
                            }
                        }
                    }
                }

                if (errorCounts.empty()) {
                    std::cout << "  No specific errors found on sub-shapes, but the global check failed." << std::endl;
                } else {
                    std::cout << "  Error Summary:" << std::endl;
                    for (const auto& pair : errorCounts) {
                        std::cout << "    - " << pair.first << ": " << pair.second << " instance(s)." << std::endl;
                    }
                }
            }
        }
        catch (const Standard_Failure& e) {
            std::cout << "Validity Check: CRITICAL OCCT EXCEPTION - " << e.GetMessageString() << std::endl;
        }

        std::cout << "=== END ANALYSIS ===" << std::endl << std::endl;
    }
    // --- END OF MODIFIED AnalyzeShape FUNCTION ---

    
    bool Scene::ExtrudeFace(uint64_t objectId, const std::vector<size_t>& faceTriangleIndices, const glm::vec3& direction, float distance, bool disableMerge) {
        SceneObject* obj = get_object_by_id(objectId);
        if (!obj || !obj->has_shape() || faceTriangleIndices.empty()) {
            return false;
        }

        const auto& mesh = obj->get_mesh_buffers();
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
        
        return ExtrudeFace(objectId, faceVertices, direction, distance, disableMerge);
    }

    bool Scene::ExtrudeFace(uint64_t objectId, const std::vector<glm::vec3>& faceVertices, const glm::vec3& direction, float distance, bool disableMerge) {
        SceneObject* obj = get_object_by_id(objectId);
        if (!obj || !obj->has_shape() || faceVertices.empty()) {
            return false;
        }
        if (std::abs(distance) < 1e-4) {
            return true;
        }

        const TopoDS_Shape* originalShape = obj->get_shape();
        
        TopoDS_Face faceToExtrude = FindOriginalFace(obj, *originalShape, faceVertices, direction);
        if (faceToExtrude.IsNull()) {
            std::cerr << "ExtrudeFace Error: Could not find corresponding B-Rep face." << std::endl;
            return false;
        }

        try {
            BRepAdaptor_Surface surfaceAdaptor(faceToExtrude, Standard_True);
            gp_Pln plane = surfaceAdaptor.Plane();
            gp_Dir occtFaceNormal = plane.Axis().Direction();
            
            if (faceToExtrude.Orientation() == TopAbs_REVERSED) {
                occtFaceNormal.Reverse();
            }

            gp_Dir userDirection(direction.x, direction.y, direction.z);
            float dotProductWithUserDir = occtFaceNormal.Dot(userDirection);

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
            
            TopoDS_Shape finalShape;
            
            bool is3DBody = (originalShape->ShapeType() == TopAbs_SOLID || originalShape->ShapeType() == TopAbs_COMPSOLID);
            if (!is3DBody && originalShape->ShapeType() == TopAbs_COMPOUND) {
                TopExp_Explorer ex(*originalShape, TopAbs_SOLID);
                if (ex.More()) is3DBody = true;
            }
            
            if (is3DBody) {
                double dotProduct = extrudeVector.Dot(gp_Vec(occtFaceNormal));
                if (dotProduct >= -1e-6) {
                    BRepAlgoAPI_Fuse fuseAlgo(*originalShape, prismShape);
                    if (!fuseAlgo.IsDone()) { std::cerr << "OCCT Error: BRepAlgoAPI_Fuse failed." << std::endl; return false; }
                    finalShape = fuseAlgo.Shape();
                } else {
                    BRepAlgoAPI_Cut cutAlgo(*originalShape, prismShape);
                    if (!cutAlgo.IsDone()) { std::cerr << "OCCT Error: BRepAlgoAPI_Cut failed." << std::endl; return false; }
                    finalShape = cutAlgo.Shape();
                }
            } else {
                finalShape = prismShape;
            }
            
            obj->set_shape(Urbaxio::CadKernel::OCCT_ShapeUniquePtr(new TopoDS_Shape(finalShape)));

            if (obj->has_shape()) {
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
        std::cout << "\n--- FINAL VERIFICATION ---\n" << std::endl;
        
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

        std::cout << "\n--- Final Object Analysis ---\n" << std::endl;
        for (const auto& [id, obj_ptr] : objects_) {
            AnalyzeShape(*obj_ptr->get_shape(), "Final Object " + std::to_string(id));
        }
        
        std::cout << "\n--- FACE SPLITTING TEST FINISHED ---\n" << std::endl;
    }

    SceneObject* Scene::FindObjectByFace(const std::vector<glm::vec3>& faceVertices) {
        if (faceVertices.empty()) return nullptr;
        
        for (auto* obj : get_all_objects()) {
            if (!obj || !obj->has_shape()) continue;
            // Guide normal doesn't matter much here, we're just checking for containment.
            TopoDS_Face foundFace = FindOriginalFace(obj, *obj->get_shape(), faceVertices, glm::vec3(0,0,1));
            if (!foundFace.IsNull()) {
                return obj;
            }
        }
        return nullptr;
    }

    void Scene::RebuildObjectByMovingVertices(
        uint64_t objectId,
        Engine::SubObjectType type,
        const std::vector<glm::vec3>& initialPositions,
        const glm::vec3& translation
    ) {
        SceneObject* obj = get_object_by_id(objectId);
        if (!obj || !obj->has_shape()) return;

        if (type == SubObjectType::FACE) {
            if (glm::length2(translation) < 1.0e-10f) {
                return;
            }

            glm::vec3 v0 = initialPositions[0];
            glm::vec3 v1 = initialPositions[1];
            glm::vec3 v2 = initialPositions[2];
            glm::vec3 faceNormal = glm::normalize(glm::cross(v1 - v0, v2 - v0));
            float projection = glm::dot(translation, faceNormal);

            if (std::fabs(projection) > 1.0e-5f) {
                glm::vec3 direction = projection >= 0.0f ? faceNormal : -faceNormal;
                float distance = std::fabs(projection);
            if (ExtrudeFace(objectId, initialPositions, direction, distance, false)) {
                    return;
                }
                std::cerr << "MoveFace warning: ExtrudeFace fallback failed, trying direct reshape." << std::endl;
            }

            const TopoDS_Shape& originalShape = *obj->get_shape();
            if (originalShape.IsNull()) {
                std::cerr << "Rebuild Error: Object has no B-Rep shape." << std::endl;
                return;
            }

            TopoDS_Face faceToMove = FindOriginalFace(obj, originalShape, initialPositions, translation);
            if (faceToMove.IsNull()) {
                std::cerr << "Rebuild Error: Could not locate face to move." << std::endl;
                return;
            }

            gp_Trsf moveTransform;
            moveTransform.SetTranslation(gp_Vec(translation.x, translation.y, translation.z));

            Handle(ShapeBuild_ReShape) reshaper = new ShapeBuild_ReShape();

            TopTools_IndexedMapOfShape vertexMap;
            TopExp::MapShapes(faceToMove, TopAbs_VERTEX, vertexMap);
            for (int i = 1; i <= vertexMap.Extent(); ++i) {
                const TopoDS_Vertex& v = TopoDS::Vertex(vertexMap(i));
                TopoDS_Shape transformedVertex = BRepBuilderAPI_Transform(v, moveTransform, Standard_True).Shape();
                reshaper->Replace(v, transformedVertex);
            }

            TopTools_IndexedMapOfShape edgeMap;
            TopExp::MapShapes(faceToMove, TopAbs_EDGE, edgeMap);
            for (int i = 1; i <= edgeMap.Extent(); ++i) {
                const TopoDS_Edge& e = TopoDS::Edge(edgeMap(i));
                TopoDS_Shape transformedEdge = BRepBuilderAPI_Transform(e, moveTransform, Standard_True).Shape();
                reshaper->Replace(e, transformedEdge);
            }

            TopoDS_Shape transformedFace = BRepBuilderAPI_Transform(faceToMove, moveTransform, Standard_True).Shape();
            reshaper->Replace(faceToMove, transformedFace);

            TopoDS_Shape movedShape = reshaper->Apply(originalShape);

            Standard_Real diag = 1.0;
            {
                Bnd_Box movedBox;
                BRepBndLib::Add(movedShape, movedBox);
                if (!movedBox.IsVoid()) {
                    Standard_Real sqExtent = movedBox.SquareExtent();
                    if (sqExtent > gp::Resolution()) {
                        diag = std::sqrt(sqExtent);
                    }
                }
            }
            Standard_Real scale = std::max(diag, 1.0);
            const Standard_Real prec     = std::max(1.0e-6, scale * 5.0e-8);
            const Standard_Real sewTol   = std::max(2.5e-4, scale * 2.5e-6);
            const Standard_Real maxTol   = std::max(1.0e-3, scale * 1.0e-5);

            try {
                BRepLib::BuildCurves3d(movedShape);
                BRepLib::SameParameter(movedShape, prec, Standard_True);
            } catch (const Standard_Failure& e) {
                std::cerr << "Rebuild Warning: BRepLib preparation failed: " << e.GetMessageString() << std::endl;
            }

            try {
                ShapeFix_Shape sfix(movedShape);
                sfix.SetPrecision(prec);
                sfix.SetMaxTolerance(maxTol);
                sfix.Perform();
                movedShape = sfix.Shape();
            } catch (const Standard_Failure& e) {
                std::cerr << "Rebuild Warning: ShapeFix_Shape failed: " << e.GetMessageString() << std::endl;
            }

            TopoDS_Shape sewnShape = movedShape;
            try {
                BRepBuilderAPI_Sewing sew(sewTol, Standard_True, Standard_True, Standard_True, Standard_True);
                sew.SetNonManifoldMode(Standard_False);
                sew.Add(movedShape);
                sew.Perform();
                TopoDS_Shape sewed = sew.SewedShape();
                if (!sewed.IsNull()) {
                    sewnShape = sewed;
                }
            } catch (const Standard_Failure& e) {
                std::cerr << "Rebuild Warning: Sewing failed: " << e.GetMessageString() << std::endl;
            }

            TopoDS_Shape solidShape = sewnShape;
            try {
                int shellCount = 0;
                for (TopExp_Explorer ex(sewnShape, TopAbs_SHELL); ex.More(); ex.Next()) shellCount++;

                if (shellCount >= 1) {
                    TopoDS_Compound comp;
                    BRep_Builder bb;
                    bb.MakeCompound(comp);

                    bool madeAny = false;
                    for (TopExp_Explorer ex(sewnShape, TopAbs_SHELL); ex.More(); ex.Next()) {
                        const TopoDS_Shell& sh = TopoDS::Shell(ex.Current());
                        BRepBuilderAPI_MakeSolid ms(sh);
                        if (ms.IsDone()) {
                            TopoDS_Solid sd = ms.Solid();
                            bb.Add(comp, sd);
                            madeAny = true;
                        }
                    }
                    if (madeAny) solidShape = comp;
                }
            } catch (const Standard_Failure& e) {
                std::cerr << "Rebuild Warning: MakeSolid failed: " << e.GetMessageString() << std::endl;
            }

            TopoDS_Shape finalShape = solidShape;
            try {
                ShapeFix_Shape finalFix(finalShape);
                finalFix.SetPrecision(prec);
                finalFix.SetMaxTolerance(maxTol);
                finalFix.Perform();
                TopoDS_Shape healed = finalFix.Shape();

                ShapeUpgrade_UnifySameDomain unif;
                unif.Initialize(healed);
                unif.SetLinearTolerance(1.0e-4);
                unif.SetAngularTolerance(1.0e-4);
                unif.AllowInternalEdges(Standard_True);
                unif.Build();

                finalShape = unif.Shape();
            } catch (const Standard_Failure& e) {
                std::cerr << "Rebuild Error: Final Heal/Unify failed: " << e.GetMessageString() << std::endl;
                return;
            }

            BRepCheck_Analyzer analyzer(finalShape);
            if (!analyzer.IsValid()) {
                std::cerr << "Rebuild Error: Final shape invalid after face move." << std::endl;
                return;
            }

            obj->set_shape(Urbaxio::CadKernel::OCCT_ShapeUniquePtr(new TopoDS_Shape(finalShape)));
            UpdateObjectBoundary(obj);

            auto mesh = Urbaxio::CadKernel::TriangulateShape(*obj->get_shape(), -1.0, 0.35, 0.05);
            obj->set_mesh_buffers(std::move(mesh));
            obj->vao = 0;
            return;
        }

        if (type == SubObjectType::VERTEX || type == SubObjectType::EDGE) {
            std::cout << "Warning: Vertex/Edge move reconstruction is currently not implemented." << std::endl;
            return;
        }
    }

} // namespace Urbaxio