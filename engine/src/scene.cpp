// engine/src/scene.cpp
#define GLM_ENABLE_EXPERIMENTAL
#include "engine/scene.h"
#include "engine/scene_object.h"
#include "engine/geometry/IGeometry.h"
#include "engine/geometry/BRepGeometry.h"
#include "engine/geometry/MeshGeometry.h"
#include "engine/MaterialManager.h"
#include "engine/MeshManager.h"
#include <cad_kernel/cad_kernel.h>
#include <cad_kernel/MeshBuffers.h> // Required for TriangulateShape return type

#include <utility>
#include <vector>
#include <array>
#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <set>
#include <cmath>
#include <algorithm> // for std::reverse, std::remove
#include <deque> // For GrowCoplanarRegion helper
#include <limits>
#include <functional>

#include <glm/gtx/norm.hpp>
#include <glm/common.hpp> // For epsilonEqual
#include <glm/gtc/epsilon.hpp>
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
#include <BRepTools_WireExplorer.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Splitter.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <ShapeFix_Shape.hxx>
#include <Standard_Failure.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
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
#include <TopLoc_Location.hxx>
#include <Poly_Triangulation.hxx>
#include <Poly_Triangle.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx> 
#include <ShapeFix_Wire.hxx>
#include <ShapeExtend_Status.hxx>
#include <sstream>
#include <BinTools.hxx>
#include <fstream>

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
#include <ShapeFix_Shell.hxx>

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

    static bool PointOnLineSegmentInclusive(
        const glm::vec3& p,
        const glm::vec3& a,
        const glm::vec3& b,
        float tolerance = Urbaxio::SCENE_POINT_EQUALITY_TOLERANCE) {
        const glm::vec3 ab = b - a;
        const float abLenSq = glm::length2(ab);
        if (abLenSq <= tolerance * tolerance) {
            return glm::distance2(p, a) <= tolerance * tolerance;
        }

        const float t = glm::dot(p - a, ab) / abLenSq;
        if (t < -tolerance || t > 1.0f + tolerance) {
            return false;
        }

        const glm::vec3 closest = a + glm::clamp(t, 0.0f, 1.0f) * ab;
        return glm::distance2(p, closest) <= tolerance * tolerance;
    }

    static bool LineSegmentIntersectionInclusive(
        const glm::vec3& p1,
        const glm::vec3& p2,
        const glm::vec3& p3,
        const glm::vec3& p4,
        glm::vec3& outIntersectionPoint,
        float tolerance = Urbaxio::SCENE_POINT_EQUALITY_TOLERANCE) {
        const glm::vec3 d1 = p2 - p1;
        const glm::vec3 d2 = p4 - p3;
        const float d1LenSq = glm::length2(d1);
        const float d2LenSq = glm::length2(d2);
        if (d1LenSq <= tolerance * tolerance || d2LenSq <= tolerance * tolerance) {
            return false;
        }

        const glm::vec3 cross = glm::cross(d1, d2);
        const float denom = glm::length2(cross);
        if (denom <= tolerance * tolerance) {
            return false;
        }

        const glm::vec3 delta = p3 - p1;
        const float coplanarDistance = std::abs(glm::dot(delta, glm::normalize(cross)));
        if (coplanarDistance > tolerance) {
            return false;
        }

        const float t = glm::dot(glm::cross(delta, d2), cross) / denom;
        const float u = glm::dot(glm::cross(delta, d1), cross) / denom;
        if (t < -tolerance || t > 1.0f + tolerance ||
            u < -tolerance || u > 1.0f + tolerance) {
            return false;
        }

        const glm::vec3 onFirst = p1 + glm::clamp(t, 0.0f, 1.0f) * d1;
        const glm::vec3 onSecond = p3 + glm::clamp(u, 0.0f, 1.0f) * d2;
        if (glm::distance2(onFirst, onSecond) > tolerance * tolerance) {
            return false;
        }

        outIntersectionPoint = (onFirst + onSecond) * 0.5f;
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

    // --- ДОБАВЬ ЭТОТ НОВЫЙ ХЕЛПЕР ---
    static std::vector<gp_Pnt> UniqueFaceVertices(const TopoDS_Face& face) {
        std::vector<gp_Pnt> points;
        for (TopExp_Explorer vertexExp(face, TopAbs_VERTEX); vertexExp.More(); vertexExp.Next()) {
            TopoDS_Vertex vertex = TopoDS::Vertex(vertexExp.Current());
            if (vertex.IsNull()) {
                continue;
            }

            const gp_Pnt point = BRep_Tool::Pnt(vertex);
            bool exists = false;
            for (const gp_Pnt& existing : points) {
                if (point.Distance(existing) <= Precision::Confusion()) {
                    exists = true;
                    break;
                }
            }
            if (!exists) {
                points.push_back(point);
            }
        }
        return points;
    }

    static bool TryFaceNormalFromVertices(const TopoDS_Face& face, gp_Vec& normal, gp_Pnt& origin) {
        const std::vector<gp_Pnt> points = UniqueFaceVertices(face);
        if (points.size() < 3) {
            return false;
        }

        origin = points.front();
        for (size_t i = 1; i + 1 < points.size(); ++i) {
            gp_Vec candidate = gp_Vec(origin, points[i]).Crossed(gp_Vec(origin, points[i + 1]));
            if (candidate.SquareMagnitude() > 1.0e-14) {
                candidate.Normalize();
                normal = candidate;
                return true;
            }
        }
        return false;
    }

    static bool ShouldHideInternalTriangulationEdge(const TopoDS_Edge& edge, const TopTools_ListOfShape& faces) {
        if (edge.IsNull() || faces.Extent() != 2) {
            return false;
        }

        TopTools_ListIteratorOfListOfShape it(faces);
        if (!it.More()) {
            return false;
        }
        const TopoDS_Face firstFace = TopoDS::Face(it.Value());
        it.Next();
        if (!it.More()) {
            return false;
        }
        const TopoDS_Face secondFace = TopoDS::Face(it.Value());

        const std::vector<gp_Pnt> firstVertices = UniqueFaceVertices(firstFace);
        const std::vector<gp_Pnt> secondVertices = UniqueFaceVertices(secondFace);
        if (firstVertices.size() != 3 || secondVertices.size() != 3) {
            return false;
        }

        gp_Vec firstNormal;
        gp_Vec secondNormal;
        gp_Pnt firstOrigin;
        gp_Pnt secondOrigin;
        if (!TryFaceNormalFromVertices(firstFace, firstNormal, firstOrigin) ||
            !TryFaceNormalFromVertices(secondFace, secondNormal, secondOrigin)) {
            return false;
        }

        const double normalDot = std::abs(firstNormal.Dot(secondNormal));
        if (normalDot < std::cos(2.0 * M_PI / 180.0)) {
            return false;
        }

        const double planeDistance = std::abs(gp_Vec(firstOrigin, secondOrigin).Dot(firstNormal));
        return planeDistance <= 1.0e-5;
    }

    std::vector<std::pair<glm::vec3, glm::vec3>> ExtractEdgesFromBRepShape(const TopoDS_Shape& shape) {
        std::vector<std::pair<glm::vec3, glm::vec3>> edges;
        TopTools_IndexedDataMapOfShapeListOfShape edgeFaceMap;
        TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeFaceMap);

        for (int edgeIndex = 1; edgeIndex <= edgeFaceMap.Extent(); ++edgeIndex) {
            const TopoDS_Edge& edge = TopoDS::Edge(edgeFaceMap.FindKey(edgeIndex));
            const TopTools_ListOfShape& faces = edgeFaceMap.FindFromIndex(edgeIndex);
            if (ShouldHideInternalTriangulationEdge(edge, faces)) {
                continue;
            }

            TopoDS_Vertex v1, v2;
            TopExp::Vertices(edge, v1, v2, Standard_True);
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

} // конец анонимного namespace

namespace Urbaxio::Engine { // начало namespace Urbaxio::Engine

    static double ShapeMatchTolerance(const TopoDS_Shape& shape) {
        double diag = 1.0;
        double maxAbsCoord = 1.0;
        try {
            Bnd_Box box;
            BRepBndLib::Add(shape, box, Standard_False);
            if (!box.IsVoid()) {
                Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
                box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
                diag = std::max(1.0, std::sqrt(std::max(0.0, static_cast<double>(box.SquareExtent()))));
                maxAbsCoord = std::max({
                    maxAbsCoord,
                    std::abs(static_cast<double>(xmin)),
                    std::abs(static_cast<double>(ymin)),
                    std::abs(static_cast<double>(zmin)),
                    std::abs(static_cast<double>(xmax)),
                    std::abs(static_cast<double>(ymax)),
                    std::abs(static_cast<double>(zmax))
                });
            }
        } catch (...) {
        }

        return std::max({1.0e-4, diag * 1.0e-6, maxAbsCoord * 1.0e-7});
    }

    static double FaceContainmentTolerance(const TopoDS_Face& face, const double baseTolerance) {
        double faceTolerance = baseTolerance;
        try {
            faceTolerance = std::max(faceTolerance, BRep_Tool::Tolerance(face));
        } catch (...) {
        }
        return std::max({faceTolerance, Precision::Confusion(), 1.0e-7});
    }

    static bool PointLiesOnFaceByUV(
        const TopoDS_Face& face,
        const gp_Pnt& point,
        const double baseTolerance) {
        if (face.IsNull()) {
            return false;
        }

        try {
            Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
            if (surface.IsNull()) {
                return false;
            }

            const double classifyTolerance = FaceContainmentTolerance(face, baseTolerance);
            const double surfaceDistanceTolerance = std::max(classifyTolerance * 50.0, 1.0e-4);

            Handle(ShapeAnalysis_Surface) analysis = new ShapeAnalysis_Surface(surface);
            gp_Pnt2d uv = analysis->ValueOfUV(point, classifyTolerance);
            gp_Pnt projected = surface->Value(uv.X(), uv.Y());
            if (projected.Distance(point) > surfaceDistanceTolerance) {
                return false;
            }

            BRepClass_FaceClassifier classifier;
            classifier.Perform(face, uv, classifyTolerance, Standard_True);
            TopAbs_State state = classifier.State();
            if (state == TopAbs_IN || state == TopAbs_ON) {
                return true;
            }

            // The UV classifier is the primary path, but keep a 3D fallback for
            // OCC edge/boundary tolerance cases where UV lands microscopically out.
            classifier.Perform(face, point, classifyTolerance, Standard_True);
            state = classifier.State();
            return state == TopAbs_IN || state == TopAbs_ON;
        } catch (const Standard_Failure&) {
            return false;
        }
    }

    static bool LoopLiesOnFaceByUV(
        const TopoDS_Face& face,
        const std::vector<glm::vec3>& loopVertices,
        const double baseTolerance) {
        if (face.IsNull() || loopVertices.size() < 3) {
            return false;
        }

        for (size_t i = 0; i < loopVertices.size(); ++i) {
            const glm::vec3& a = loopVertices[i];
            const glm::vec3& b = loopVertices[(i + 1) % loopVertices.size()];
            const glm::vec3 midpoint = (a + b) * 0.5f;

            if (!PointLiesOnFaceByUV(face, gp_Pnt(a.x, a.y, a.z), baseTolerance) ||
                !PointLiesOnFaceByUV(face, gp_Pnt(midpoint.x, midpoint.y, midpoint.z), baseTolerance)) {
                return false;
            }
        }

        return true;
    }

    static bool LineLiesOnFaceByUV(
        const TopoDS_Face& face,
        const Line& line,
        const double baseTolerance) {
        if (face.IsNull()) {
            return false;
        }

        const glm::vec3 midpoint = (line.start + line.end) * 0.5f;
        return PointLiesOnFaceByUV(face, gp_Pnt(line.start.x, line.start.y, line.start.z), baseTolerance) &&
               PointLiesOnFaceByUV(face, gp_Pnt(midpoint.x, midpoint.y, midpoint.z), baseTolerance) &&
               PointLiesOnFaceByUV(face, gp_Pnt(line.end.x, line.end.y, line.end.z), baseTolerance);
    }

    static bool LineLiesOnFacePlane(
        const TopoDS_Face& face,
        const glm::vec3& start,
        const glm::vec3& end,
        const glm::vec3& planeNormal,
        const float planeOffset,
        const double baseTolerance) {
        if (face.IsNull()) {
            return false;
        }

        try {
            BRepAdaptor_Surface adaptor(face, Standard_False);
            if (adaptor.GetType() != GeomAbs_Plane) {
                return false;
            }

            const gp_Pln plane = adaptor.Plane();
            const gp_Dir faceDir = plane.Axis().Direction();
            const glm::vec3 faceNormal(
                static_cast<float>(faceDir.X()),
                static_cast<float>(faceDir.Y()),
                static_cast<float>(faceDir.Z()));
            if (std::abs(glm::dot(glm::normalize(faceNormal), glm::normalize(planeNormal))) <
                std::cos(1.0e-4f)) {
                return false;
            }

            const gp_Pnt facePoint = plane.Location();
            const glm::vec3 facePointGlm(
                static_cast<float>(facePoint.X()),
                static_cast<float>(facePoint.Y()),
                static_cast<float>(facePoint.Z()));
            if (std::abs(glm::dot(glm::normalize(planeNormal), facePointGlm) - planeOffset) >
                static_cast<float>(std::max(baseTolerance * 50.0, 1.0e-4))) {
                return false;
            }

            const gp_Pnt p0(start.x, start.y, start.z);
            const gp_Pnt p1(end.x, end.y, end.z);
            return plane.Distance(p0) <= std::max(baseTolerance * 50.0, 1.0e-4) &&
                   plane.Distance(p1) <= std::max(baseTolerance * 50.0, 1.0e-4);
        } catch (const Standard_Failure&) {
            return false;
        }
    }

    static bool PointLiesOnFaceBoundary(
        const TopoDS_Face& face,
        const gp_Pnt& point,
        const double baseTolerance) {
        if (face.IsNull()) {
            return false;
        }

        const double tolerance = std::max({FaceContainmentTolerance(face, baseTolerance) * 50.0, 1.0e-5, baseTolerance});
        try {
            TopoDS_Vertex pointVertex = BRepBuilderAPI_MakeVertex(point).Vertex();
            for (TopExp_Explorer edgeExplorer(face, TopAbs_EDGE); edgeExplorer.More(); edgeExplorer.Next()) {
                const TopoDS_Edge edge = TopoDS::Edge(edgeExplorer.Current());
                if (edge.IsNull()) {
                    continue;
                }

                BRepExtrema_DistShapeShape distance(pointVertex, edge);
                distance.Perform();
                if (distance.IsDone() && distance.Value() <= tolerance) {
                    return true;
                }
            }
        } catch (const Standard_Failure&) {
        }

        return false;
    }

    static bool LineLiesOnFaceBoundary(
        const TopoDS_Face& face,
        const Line& line,
        const double baseTolerance) {
        const glm::vec3 midpoint = (line.start + line.end) * 0.5f;
        return PointLiesOnFaceBoundary(face, gp_Pnt(line.start.x, line.start.y, line.start.z), baseTolerance) &&
               PointLiesOnFaceBoundary(face, gp_Pnt(midpoint.x, midpoint.y, midpoint.z), baseTolerance) &&
               PointLiesOnFaceBoundary(face, gp_Pnt(line.end.x, line.end.y, line.end.z), baseTolerance);
    }

    static TopoDS_Face FindPlanarFaceContainingLine(
        const TopoDS_Shape& shape,
        const Line& line,
        const double baseTolerance) {
        for (TopExp_Explorer faceExplorer(shape, TopAbs_FACE); faceExplorer.More(); faceExplorer.Next()) {
            TopoDS_Face candidateFace = TopoDS::Face(faceExplorer.Current());
            try {
                BRepAdaptor_Surface surfaceAdaptor(candidateFace, Standard_False);
                if (surfaceAdaptor.GetType() != GeomAbs_Plane) {
                    continue;
                }
                if (LineLiesOnFaceByUV(candidateFace, line, baseTolerance)) {
                    return candidateFace;
                }
            } catch (const Standard_Failure&) {
                continue;
            }
        }
        return TopoDS_Face();
    }

    static bool MakeCuttingEdgeOnFace(
        const TopoDS_Face& face,
        const Line& line,
        const double baseTolerance,
        TopoDS_Edge& outEdge) {
        if (face.IsNull() || glm::distance2(line.start, line.end) <=
            SCENE_POINT_EQUALITY_TOLERANCE * SCENE_POINT_EQUALITY_TOLERANCE) {
            return false;
        }

        try {
            Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
            if (surface.IsNull()) {
                return false;
            }

            Handle(ShapeAnalysis_Surface) analysis = new ShapeAnalysis_Surface(surface);
            gp_Pnt startPoint(line.start.x, line.start.y, line.start.z);
            gp_Pnt endPoint(line.end.x, line.end.y, line.end.z);
            gp_Pnt2d uvStart = analysis->ValueOfUV(startPoint, baseTolerance);
            gp_Pnt2d uvEnd = analysis->ValueOfUV(endPoint, baseTolerance);
            const double uvDistance = uvStart.Distance(uvEnd);
            if (uvDistance <= Precision::Confusion()) {
                return false;
            }

            Handle(Geom2d_Line) pcurve = new Geom2d_Line(
                uvStart,
                gp_Dir2d(uvEnd.X() - uvStart.X(), uvEnd.Y() - uvStart.Y()));

            BRepBuilderAPI_MakeEdge edgeMaker(pcurve, surface, 0.0, uvDistance);
            if (!edgeMaker.IsDone() || edgeMaker.Edge().IsNull()) {
                return false;
            }

            outEdge = edgeMaker.Edge();
            BRepLib::BuildCurves3d(outEdge);
            BRepLib::SameParameter(outEdge, std::max(1.0e-7, baseTolerance * 0.1), Standard_True);
            return true;
        } catch (const Standard_Failure& e) {
            std::cerr << "MakeCuttingEdgeOnFace failed: " << e.GetMessageString() << std::endl;
            return false;
        }
    }

    static void AddUniqueFaceCandidate(std::vector<TopoDS_Face>& candidates, const TopoDS_Face& face) {
        if (face.IsNull()) {
            return;
        }
        for (const auto& existing : candidates) {
            if (existing.IsSame(face)) {
                return;
            }
        }
        candidates.push_back(face);
    }

    static const char* SubObjectTypeName(SubObjectType type) {
        switch (type) {
            case SubObjectType::VERTEX: return "VERTEX";
            case SubObjectType::EDGE: return "EDGE";
            case SubObjectType::FACE: return "FACE";
            default: return "UNKNOWN";
        }
    }

    static gp_Pnt TranslatedPoint(const gp_Pnt& p, const gp_Vec& translation) {
        gp_Pnt moved = p;
        moved.Translate(translation);
        return moved;
    }

    static bool MapContainsSameShape(const TopTools_IndexedMapOfShape& map, const TopoDS_Shape& shape) {
        return !shape.IsNull() && map.FindIndex(shape) > 0;
    }

    static gp_Pnt LocatedVertexPoint(const TopoDS_Vertex& vertex) {
        gp_Pnt point = BRep_Tool::Pnt(vertex);
        try {
            TopLoc_Location location = vertex.Location();
            if (!location.IsIdentity()) {
                point.Transform(location.Transformation());
            }
        } catch (...) {
        }
        return point;
    }

    static bool PointMatchesAnyPosition(const gp_Pnt& point, const std::vector<glm::vec3>& positions, const double tolerance) {
        for (const auto& pos : positions) {
            if (point.Distance(gp_Pnt(pos.x, pos.y, pos.z)) <= tolerance) {
                return true;
            }
        }
        return false;
    }

    static bool VertexMatchesAnyPosition(const TopoDS_Vertex& vertex, const std::vector<glm::vec3>& positions, const double tolerance) {
        if (vertex.IsNull()) {
            return false;
        }

        const gp_Pnt rawPoint = BRep_Tool::Pnt(vertex);
        if (PointMatchesAnyPosition(rawPoint, positions, tolerance)) {
            return true;
        }

        const gp_Pnt locatedPoint = LocatedVertexPoint(vertex);
        if (!locatedPoint.IsEqual(rawPoint, Precision::Confusion()) &&
            PointMatchesAnyPosition(locatedPoint, positions, tolerance)) {
            return true;
        }

        return false;
    }

    static bool IsVertexSelected(
        const TopoDS_Vertex& vertex,
        const TopTools_IndexedMapOfShape& selectedVertices,
        const std::vector<glm::vec3>& selectedPositions,
        const double tolerance) {
        if (vertex.IsNull()) {
            return false;
        }
        if (MapContainsSameShape(selectedVertices, vertex)) {
            return true;
        }
        return VertexMatchesAnyPosition(vertex, selectedPositions, tolerance);
    }

    static TopTools_IndexedMapOfShape FindVerticesNearPositions(
        const TopoDS_Shape& shape,
        const std::vector<glm::vec3>& positions,
        const double tolerance) {
        TopTools_IndexedMapOfShape result;
        if (positions.empty()) return result;

        for (TopExp_Explorer vertexExp(shape, TopAbs_VERTEX); vertexExp.More(); vertexExp.Next()) {
            TopoDS_Vertex vertex = TopoDS::Vertex(vertexExp.Current());
            if (VertexMatchesAnyPosition(vertex, positions, tolerance)) {
                result.Add(vertex);
            }
        }
        return result;
    }

    static void AddVerticesFromMap(TopTools_IndexedMapOfShape& destination, const TopTools_IndexedMapOfShape& source) {
        for (int i = 1; i <= source.Extent(); ++i) {
            destination.Add(source(i));
        }
    }

    static void AddUniquePosition(std::vector<glm::vec3>& positions, const glm::vec3& candidate, const float tolerance) {
        const float toleranceSq = tolerance * tolerance;
        for (const auto& existing : positions) {
            if (glm::distance2(existing, candidate) <= toleranceSq) {
                return;
            }
        }
        positions.push_back(candidate);
    }

    static std::vector<glm::vec3> VertexPositionsOfShape(const TopoDS_Shape& shape) {
        std::vector<glm::vec3> positions;
        TopTools_IndexedMapOfShape uniqueVertices;
        TopExp::MapShapes(shape, TopAbs_VERTEX, uniqueVertices);
        positions.reserve(static_cast<size_t>(uniqueVertices.Extent()) * 2);

        auto addUnique = [&positions](const gp_Pnt& p) {
            glm::vec3 candidate(static_cast<float>(p.X()), static_cast<float>(p.Y()), static_cast<float>(p.Z()));
            for (const auto& existing : positions) {
                if (glm::distance2(existing, candidate) <= 1.0e-12f) {
                    return;
                }
            }
            positions.push_back(candidate);
        };

        for (int i = 1; i <= uniqueVertices.Extent(); ++i) {
            TopoDS_Vertex vertex = TopoDS::Vertex(uniqueVertices(i));
            const gp_Pnt rawPoint = BRep_Tool::Pnt(vertex);
            addUnique(rawPoint);
            const gp_Pnt locatedPoint = LocatedVertexPoint(vertex);
            if (!locatedPoint.IsEqual(rawPoint, Precision::Confusion())) {
                addUnique(locatedPoint);
            }
        }
        return positions;
    }

    static bool FaceUsesAnySelectedVertex(
        const TopoDS_Face& face,
        const TopTools_IndexedMapOfShape& selectedVertices,
        const std::vector<glm::vec3>& selectedPositions,
        const double tolerance) {
        for (TopExp_Explorer vertexExp(face, TopAbs_VERTEX); vertexExp.More(); vertexExp.Next()) {
            if (IsVertexSelected(TopoDS::Vertex(vertexExp.Current()), selectedVertices, selectedPositions, tolerance)) {
                return true;
            }
        }
        return false;
    }

    static TopoDS_Edge BuildMovedEdge(
        const TopoDS_Edge& edge,
        const TopTools_IndexedMapOfShape& selectedVertices,
        const gp_Vec& translation,
        bool& changed) {
        changed = false;

        TopoDS_Vertex v1, v2;
        TopExp::Vertices(edge, v1, v2, Standard_True);
        if (v1.IsNull() || v2.IsNull()) {
            return edge;
        }

        const bool moveV1 = MapContainsSameShape(selectedVertices, v1);
        const bool moveV2 = MapContainsSameShape(selectedVertices, v2);
        if (!moveV1 && !moveV2) {
            return edge;
        }

        changed = true;

        if (moveV1 && moveV2) {
            gp_Trsf moveTransform;
            moveTransform.SetTranslation(translation);
            TopoDS_Shape transformed = BRepBuilderAPI_Transform(edge, moveTransform, Standard_True).Shape();
            if (!transformed.IsNull() && transformed.ShapeType() == TopAbs_EDGE) {
                TopoDS_Edge movedEdge = TopoDS::Edge(transformed);
                movedEdge.Orientation(TopAbs_FORWARD);
                return movedEdge;
            }
        }

        gp_Pnt p1 = BRep_Tool::Pnt(v1);
        gp_Pnt p2 = BRep_Tool::Pnt(v2);
        if (moveV1) p1 = TranslatedPoint(p1, translation);
        if (moveV2) p2 = TranslatedPoint(p2, translation);
        if (p1.IsEqual(p2, Precision::Confusion())) {
            return TopoDS_Edge();
        }

        BRepBuilderAPI_MakeEdge edgeMaker(p1, p2);
        if (!edgeMaker.IsDone()) {
            return TopoDS_Edge();
        }

        TopoDS_Edge movedEdge = edgeMaker.Edge();
        movedEdge.Orientation(TopAbs_FORWARD);
        return movedEdge;
    }

    static bool TryMakePlaneFromPoints(const std::vector<gp_Pnt>& points, const double tolerance, gp_Pln& plane) {
        if (points.size() < 3) return false;

        for (size_t i = 0; i < points.size(); ++i) {
            for (size_t j = i + 1; j < points.size(); ++j) {
                gp_Vec a(points[i], points[j]);
                if (a.SquareMagnitude() <= tolerance * tolerance) continue;

                for (size_t k = j + 1; k < points.size(); ++k) {
                    gp_Vec b(points[i], points[k]);
                    gp_Vec n = a.Crossed(b);
                    if (n.SquareMagnitude() <= tolerance * tolerance) continue;

                    try {
                        plane = gp_Pln(points[i], gp_Dir(n));
                    } catch (...) {
                        return false;
                    }

                    for (const gp_Pnt& p : points) {
                        if (plane.Distance(p) > tolerance) {
                            return false;
                        }
                    }
                    return true;
                }
            }
        }
        return false;
    }

    static std::vector<gp_Pnt> RemoveConsecutiveDuplicatePoints(const std::vector<gp_Pnt>& points, const double tolerance) {
        std::vector<gp_Pnt> cleaned;
        cleaned.reserve(points.size());

        for (const gp_Pnt& point : points) {
            if (cleaned.empty() || !point.IsEqual(cleaned.back(), tolerance)) {
                cleaned.push_back(point);
            }
        }

        if (cleaned.size() > 1 && cleaned.front().IsEqual(cleaned.back(), tolerance)) {
            cleaned.pop_back();
        }

        return cleaned;
    }

    static bool BuildPolygonWireFromOrderedPoints(
        const std::vector<gp_Pnt>& points,
        const double tolerance,
        TopoDS_Wire& outWire) {
        outWire = TopoDS_Wire();

        const std::vector<gp_Pnt> cleaned = RemoveConsecutiveDuplicatePoints(points, tolerance);
        if (cleaned.size() < 3) {
            return false;
        }

        try {
            BRepBuilderAPI_MakeWire wireMaker;
            for (size_t i = 0; i < cleaned.size(); ++i) {
                const gp_Pnt& a = cleaned[i];
                const gp_Pnt& b = cleaned[(i + 1) % cleaned.size()];
                if (a.IsEqual(b, tolerance)) {
                    continue;
                }

                BRepBuilderAPI_MakeEdge edgeMaker(a, b);
                if (!edgeMaker.IsDone()) {
                    return false;
                }
                wireMaker.Add(edgeMaker.Edge());
            }

            if (!wireMaker.IsDone()) {
                return false;
            }

            TopoDS_Wire rawWire = wireMaker.Wire();
            Handle(ShapeFix_Wire) wireFix = new ShapeFix_Wire();
            wireFix->Load(rawWire);
            wireFix->SetPrecision(std::max(Precision::Confusion(), tolerance));
            wireFix->SetMaxTolerance(std::max(tolerance * 20.0, 1.0e-5));
            wireFix->SetMinTolerance(std::max(Precision::Confusion(), tolerance * 0.1));
            wireFix->ClosedWireMode() = Standard_True;
            wireFix->FixReorder();
            wireFix->FixConnected();
            wireFix->FixClosed();
            wireFix->Perform();

            outWire = wireFix->Wire();
            return !outWire.IsNull();
        } catch (const Standard_Failure&) {
            return false;
        }
    }

    static bool BuildMovedWire(
        const TopoDS_Wire& wire,
        const TopoDS_Face& face,
        const TopTools_IndexedMapOfShape& selectedVertices,
        const std::vector<glm::vec3>& selectedPositions,
        const gp_Vec& translation,
        const double tolerance,
        TopoDS_Wire& movedWire,
        std::vector<TopoDS_Edge>& movedEdges,
        std::vector<gp_Pnt>& orderedPoints,
        bool& changed) {
        changed = false;
        movedEdges.clear();
        orderedPoints.clear();

        BRepBuilderAPI_MakeWire wireMaker;
        for (BRepTools_WireExplorer edgeExp(wire, face); edgeExp.More(); edgeExp.Next()) {
            TopoDS_Edge originalEdge = edgeExp.Current();
            TopoDS_Vertex vStart, vEnd;
            TopExp::Vertices(originalEdge, vStart, vEnd, Standard_True);
            if (vStart.IsNull() || vEnd.IsNull()) {
                return false;
            }

            gp_Pnt pStart = BRep_Tool::Pnt(vStart);
            gp_Pnt pEnd = BRep_Tool::Pnt(vEnd);
            const bool moveStart = IsVertexSelected(vStart, selectedVertices, selectedPositions, tolerance);
            const bool moveEnd = IsVertexSelected(vEnd, selectedVertices, selectedPositions, tolerance);
            if (moveStart) pStart = TranslatedPoint(pStart, translation);
            if (moveEnd) pEnd = TranslatedPoint(pEnd, translation);
            changed = changed || moveStart || moveEnd;

            if (!pStart.IsEqual(pEnd, tolerance)) {
                BRepBuilderAPI_MakeEdge edgeMaker(pStart, pEnd);
                if (edgeMaker.IsDone()) {
                    TopoDS_Edge movedEdge = edgeMaker.Edge();
                    movedEdge.Orientation(TopAbs_FORWARD);
                    wireMaker.Add(movedEdge);
                    movedEdges.push_back(movedEdge);
                }
            }
            orderedPoints.push_back(pStart);
        }

        orderedPoints = RemoveConsecutiveDuplicatePoints(orderedPoints, tolerance);
        if (orderedPoints.size() < 3) {
            return false;
        }

        if (!movedEdges.empty() && wireMaker.IsDone()) {
            movedWire = wireMaker.Wire();
            if (!movedWire.IsNull()) {
                return true;
            }
        }

        return BuildPolygonWireFromOrderedPoints(orderedPoints, tolerance, movedWire);
    }

    static TopoDS_Face BuildTriangleFace(const gp_Pnt& a, const gp_Pnt& b, const gp_Pnt& c, TopAbs_Orientation orientation) {
        try {
            if (a.IsEqual(b, Precision::Confusion()) ||
                b.IsEqual(c, Precision::Confusion()) ||
                c.IsEqual(a, Precision::Confusion())) {
                return TopoDS_Face();
            }

            BRepBuilderAPI_MakeWire wireMaker;
            wireMaker.Add(BRepBuilderAPI_MakeEdge(a, b));
            wireMaker.Add(BRepBuilderAPI_MakeEdge(b, c));
            wireMaker.Add(BRepBuilderAPI_MakeEdge(c, a));
            if (!wireMaker.IsDone()) {
                return TopoDS_Face();
            }

            BRepBuilderAPI_MakeFace faceMaker(wireMaker.Wire(), Standard_True);
            if (!faceMaker.IsDone()) {
                return TopoDS_Face();
            }

            TopoDS_Face face = faceMaker.Face();
            face.Orientation(orientation);
            return face;
        } catch (const Standard_Failure&) {
            return TopoDS_Face();
        }
    }

    struct TriangleIndex {
        int a = 0;
        int b = 0;
        int c = 0;
    };

    struct MeshEdgeKey {
        int a = 0;
        int b = 0;

        MeshEdgeKey() = default;
        MeshEdgeKey(int first, int second)
            : a(std::min(first, second)), b(std::max(first, second)) {}

        bool operator<(const MeshEdgeKey& other) const {
            if (a != other.a) {
                return a < other.a;
            }
            return b < other.b;
        }
    };

    struct EdgeUse {
        int triangleIndex = -1;
        int thirdVertex = -1;
    };

    struct ProjectedPoint {
        double x = 0.0;
        double y = 0.0;
    };

    static bool BuildProjectionBasis(
        const std::vector<gp_Pnt>& points,
        const std::array<int, 4>& indices,
        gp_Dir& outU,
        gp_Dir& outV) {
        constexpr double eps = 1.0e-12;
        const gp_Pnt& p0 = points[indices[0]];
        const gp_Pnt& p1 = points[indices[1]];
        const gp_Pnt& p2 = points[indices[2]];
        const gp_Pnt& p3 = points[indices[3]];

        std::array<gp_Vec, 4> edges = {
            gp_Vec(p0, p1),
            gp_Vec(p1, p2),
            gp_Vec(p2, p3),
            gp_Vec(p3, p0)
        };

        gp_Vec normal(0.0, 0.0, 0.0);
        normal.Add(edges[0].Crossed(edges[1]));
        normal.Add(edges[1].Crossed(edges[2]));
        normal.Add(edges[2].Crossed(edges[3]));
        normal.Add(edges[3].Crossed(edges[0]));
        if (normal.SquareMagnitude() <= eps) {
            normal = gp_Vec(p0, p1).Crossed(gp_Vec(p0, p2));
        }
        if (normal.SquareMagnitude() <= eps) {
            return false;
        }
        normal.Normalize();

        gp_Vec u = gp_Vec(p0, p1);
        double bestLength = u.SquareMagnitude();
        for (const gp_Vec& candidate : edges) {
            const double length = candidate.SquareMagnitude();
            if (length > bestLength) {
                u = candidate;
                bestLength = length;
            }
        }
        if (bestLength <= eps) {
            return false;
        }

        u.Subtract(normal.Multiplied(u.Dot(normal)));
        if (u.SquareMagnitude() <= eps) {
            return false;
        }
        u.Normalize();

        gp_Vec v = normal.Crossed(u);
        if (v.SquareMagnitude() <= eps) {
            return false;
        }
        v.Normalize();

        outU = gp_Dir(u);
        outV = gp_Dir(v);
        return true;
    }

    static ProjectedPoint ProjectToBasis(const gp_Pnt& origin, const gp_Dir& u, const gp_Dir& v, const gp_Pnt& point) {
        const gp_Vec offset(origin, point);
        return {offset.Dot(gp_Vec(u)), offset.Dot(gp_Vec(v))};
    }

    static double Cross2D(const ProjectedPoint& a, const ProjectedPoint& b, const ProjectedPoint& c) {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    }

    static double SquareDistance2D(const ProjectedPoint& a, const ProjectedPoint& b) {
        const double dx = a.x - b.x;
        const double dy = a.y - b.y;
        return dx * dx + dy * dy;
    }

    static double TriangleQuality2D(const ProjectedPoint& a, const ProjectedPoint& b, const ProjectedPoint& c) {
        const double area2 = std::abs(Cross2D(a, b, c));
        const double perimeterSq =
            SquareDistance2D(a, b) +
            SquareDistance2D(b, c) +
            SquareDistance2D(c, a);
        if (area2 <= 1.0e-12 || perimeterSq <= 1.0e-12) {
            return 0.0;
        }
        return area2 / perimeterSq;
    }

    static TriangleIndex OrientedTriangle(
        int a,
        int b,
        int c,
        const double desiredSign,
        const std::vector<ProjectedPoint>& projected) {
        TriangleIndex triangle{a, b, c};
        const double currentSign = Cross2D(projected[a], projected[b], projected[c]);
        if (desiredSign != 0.0 && currentSign * desiredSign < 0.0) {
            std::swap(triangle.b, triangle.c);
        }
        return triangle;
    }

    static bool TryFlipInternalEdge(
        const MeshEdgeKey& edge,
        const EdgeUse& firstUse,
        const EdgeUse& secondUse,
        const std::map<MeshEdgeKey, std::vector<EdgeUse>>& edgeUses,
        const std::vector<gp_Pnt>& points,
        std::vector<TriangleIndex>& triangles) {
        const int a = edge.a;
        const int b = edge.b;
        const int c = firstUse.thirdVertex;
        const int d = secondUse.thirdVertex;
        if (a == b || c == d || c == a || c == b || d == a || d == b) {
            return false;
        }

        if (edgeUses.find(MeshEdgeKey(c, d)) != edgeUses.end()) {
            return false;
        }

        gp_Dir u(1.0, 0.0, 0.0);
        gp_Dir v(0.0, 1.0, 0.0);
        if (!BuildProjectionBasis(points, {a, c, b, d}, u, v)) {
            return false;
        }

        const gp_Pnt& origin = points[a];
        std::vector<ProjectedPoint> projected(points.size());
        projected[a] = ProjectToBasis(origin, u, v, points[a]);
        projected[b] = ProjectToBasis(origin, u, v, points[b]);
        projected[c] = ProjectToBasis(origin, u, v, points[c]);
        projected[d] = ProjectToBasis(origin, u, v, points[d]);

        const double sideC = Cross2D(projected[a], projected[b], projected[c]);
        const double sideD = Cross2D(projected[a], projected[b], projected[d]);
        const double sideA = Cross2D(projected[c], projected[d], projected[a]);
        const double sideB = Cross2D(projected[c], projected[d], projected[b]);
        const double sideEps = 1.0e-10;
        if (sideC * sideD >= -sideEps || sideA * sideB >= -sideEps) {
            return false;
        }

        const TriangleIndex oldFirst = triangles[firstUse.triangleIndex];
        const TriangleIndex oldSecond = triangles[secondUse.triangleIndex];
        const double oldSignFirst = Cross2D(projected[oldFirst.a], projected[oldFirst.b], projected[oldFirst.c]);
        const double oldSignSecond = Cross2D(projected[oldSecond.a], projected[oldSecond.b], projected[oldSecond.c]);
        const double oldQuality = std::min(
            TriangleQuality2D(projected[oldFirst.a], projected[oldFirst.b], projected[oldFirst.c]),
            TriangleQuality2D(projected[oldSecond.a], projected[oldSecond.b], projected[oldSecond.c]));
        const double newQuality = std::min(
            TriangleQuality2D(projected[c], projected[d], projected[a]),
            TriangleQuality2D(projected[c], projected[b], projected[d]));
        if (newQuality <= 0.0) {
            return false;
        }

        const double oldDiagSq = points[a].SquareDistance(points[b]);
        const double newDiagSq = points[c].SquareDistance(points[d]);
        const bool qualityImproves = newQuality > oldQuality * 1.0001;
        const bool shorterWithoutRealQualityLoss =
            newQuality >= oldQuality * 0.997 &&
            newDiagSq + 1.0e-10 < oldDiagSq;
        if (!qualityImproves && !shorterWithoutRealQualityLoss) {
            return false;
        }

        triangles[firstUse.triangleIndex] = OrientedTriangle(c, d, a, oldSignFirst, projected);
        triangles[secondUse.triangleIndex] = OrientedTriangle(c, b, d, oldSignSecond, projected);
        return true;
    }

    static int OptimizeInternalTriangulationEdges(
        const std::vector<gp_Pnt>& points,
        std::vector<TriangleIndex>& triangles) {
        if (points.size() < 4 || triangles.size() < 2) {
            return 0;
        }

        int totalFlips = 0;
        const size_t maxPasses = std::max<size_t>(8, triangles.size() * 4);
        for (size_t pass = 0; pass < maxPasses; ++pass) {
            std::map<MeshEdgeKey, std::vector<EdgeUse>> edgeUses;
            for (size_t triIndex = 0; triIndex < triangles.size(); ++triIndex) {
                const TriangleIndex& tri = triangles[triIndex];
                if (tri.a == tri.b || tri.b == tri.c || tri.c == tri.a) {
                    continue;
                }
                edgeUses[MeshEdgeKey(tri.a, tri.b)].push_back({static_cast<int>(triIndex), tri.c});
                edgeUses[MeshEdgeKey(tri.b, tri.c)].push_back({static_cast<int>(triIndex), tri.a});
                edgeUses[MeshEdgeKey(tri.c, tri.a)].push_back({static_cast<int>(triIndex), tri.b});
            }

            bool flipped = false;
            for (const auto& [edge, uses] : edgeUses) {
                if (uses.size() != 2) {
                    continue;
                }
                if (TryFlipInternalEdge(edge, uses[0], uses[1], edgeUses, points, triangles)) {
                    ++totalFlips;
                    flipped = true;
                    break;
                }
            }

            if (!flipped) {
                break;
            }
        }

        return totalFlips;
    }

    static TopoDS_Shape BuildTriangulatedFaceCompound(
        const std::vector<gp_Pnt>& orderedPoints,
        TopAbs_Orientation orientation) {
        if (orderedPoints.size() < 3) {
            return TopoDS_Shape();
        }

        const std::vector<gp_Pnt> cleanedPoints = RemoveConsecutiveDuplicatePoints(orderedPoints, Precision::Confusion());
        if (cleanedPoints.size() < 3) {
            return TopoDS_Shape();
        }

        TopoDS_Compound compound;
        BRep_Builder builder;
        builder.MakeCompound(compound);
        bool addedAny = false;

        auto addTriangle = [&](const gp_Pnt& a, const gp_Pnt& b, const gp_Pnt& c) {
            TopoDS_Face tri = BuildTriangleFace(a, b, c, orientation);
            if (tri.IsNull()) {
                return false;
            }
            builder.Add(compound, tri);
            addedAny = true;
            return true;
        };

        if (cleanedPoints.size() == 4) {
            const double diag02 = cleanedPoints[0].SquareDistance(cleanedPoints[2]);
            const double diag13 = cleanedPoints[1].SquareDistance(cleanedPoints[3]);
            if (diag13 < diag02) {
                if (!addTriangle(cleanedPoints[0], cleanedPoints[1], cleanedPoints[3]) ||
                    !addTriangle(cleanedPoints[1], cleanedPoints[2], cleanedPoints[3])) {
                    return TopoDS_Shape();
                }
            } else {
                if (!addTriangle(cleanedPoints[0], cleanedPoints[1], cleanedPoints[2]) ||
                    !addTriangle(cleanedPoints[0], cleanedPoints[2], cleanedPoints[3])) {
                    return TopoDS_Shape();
                }
            }
            return addedAny ? TopoDS_Shape(compound) : TopoDS_Shape();
        }

        for (size_t i = 1; i + 1 < cleanedPoints.size(); ++i) {
            if (!addTriangle(cleanedPoints[0], cleanedPoints[i], cleanedPoints[i + 1])) {
                return TopoDS_Shape();
            }
        }

        return addedAny ? TopoDS_Shape(compound) : TopoDS_Shape();
    }

    static bool PointIsSelectedForMove(
        const gp_Pnt& point,
        const TopTools_IndexedMapOfShape& selectedVertices,
        const std::vector<glm::vec3>& selectedPositions,
        const double tolerance) {
        if (PointMatchesAnyPosition(point, selectedPositions, tolerance)) {
            return true;
        }

        for (int i = 1; i <= selectedVertices.Extent(); ++i) {
            TopoDS_Vertex vertex = TopoDS::Vertex(selectedVertices(i));
            if (vertex.IsNull()) {
                continue;
            }

            const gp_Pnt rawPoint = BRep_Tool::Pnt(vertex);
            if (point.Distance(rawPoint) <= tolerance) {
                return true;
            }

            const gp_Pnt locatedPoint = LocatedVertexPoint(vertex);
            if (point.Distance(locatedPoint) <= tolerance) {
                return true;
            }
        }

        return false;
    }

    static TopoDS_Shape BuildTriangulatedMovedFaceFromMesh(
        TopoDS_Face originalFace,
        const TopTools_IndexedMapOfShape& selectedVertices,
        const std::vector<glm::vec3>& selectedPositions,
        const gp_Vec& translation,
        const double tolerance,
        bool& changed) {
        changed = false;

        try {
            TopLoc_Location location;
            Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(originalFace, location);
            if (triangulation.IsNull() || triangulation->NbTriangles() == 0) {
                BRepMesh_IncrementalMesh mesher(originalFace, std::max(1.0e-4, tolerance * 10.0));
                mesher.Perform();
                triangulation = BRep_Tool::Triangulation(originalFace, location);
            }
            if (triangulation.IsNull() || triangulation->NbTriangles() == 0) {
                return TopoDS_Shape();
            }

            const gp_Trsf locationTransform = location.Transformation();
            std::vector<gp_Pnt> movedNodes;
            movedNodes.reserve(static_cast<size_t>(triangulation->NbNodes()));
            for (int nodeIndex = 1; nodeIndex <= triangulation->NbNodes(); ++nodeIndex) {
                gp_Pnt point = triangulation->Node(nodeIndex).Transformed(locationTransform);
                if (PointIsSelectedForMove(point, selectedVertices, selectedPositions, tolerance)) {
                    point = TranslatedPoint(point, translation);
                    changed = true;
                }
                movedNodes.push_back(point);
            }

            std::vector<TriangleIndex> triangles;
            triangles.reserve(static_cast<size_t>(triangulation->NbTriangles()));
            for (int triangleIndex = 1; triangleIndex <= triangulation->NbTriangles(); ++triangleIndex) {
                int n1 = 0;
                int n2 = 0;
                int n3 = 0;
                triangulation->Triangle(triangleIndex).Get(n1, n2, n3);
                const int i1 = n1 - 1;
                const int i2 = n2 - 1;
                const int i3 = n3 - 1;
                if (i1 < 0 || i2 < 0 || i3 < 0 ||
                    i1 >= static_cast<int>(movedNodes.size()) ||
                    i2 >= static_cast<int>(movedNodes.size()) ||
                    i3 >= static_cast<int>(movedNodes.size())) {
                    return TopoDS_Shape();
                }
                triangles.push_back({i1, i2, i3});
            }

            const int edgeFlips = OptimizeInternalTriangulationEdges(movedNodes, triangles);
            if (edgeFlips > 0) {
                std::cout << "MoveSubObject: optimized fallback triangulation with "
                          << edgeFlips << " edge flips." << std::endl;
            }

            TopoDS_Compound triangleCompound;
            BRep_Builder builder;
            builder.MakeCompound(triangleCompound);

            bool addedAny = false;
            for (const TriangleIndex& triangle : triangles) {
                TopoDS_Face triangleFace = BuildTriangleFace(
                    movedNodes[triangle.a],
                    movedNodes[triangle.b],
                    movedNodes[triangle.c],
                    originalFace.Orientation());
                if (triangleFace.IsNull()) {
                    return TopoDS_Shape();
                }
                builder.Add(triangleCompound, triangleFace);
                addedAny = true;
            }

            return addedAny ? TopoDS_Shape(triangleCompound) : TopoDS_Shape();
        } catch (const Standard_Failure& e) {
            std::cout << "MoveSubObject Warning: triangulated face fallback failed: "
                      << e.GetMessageString() << std::endl;
            return TopoDS_Shape();
        }
    }

    static TopoDS_Shape BuildMovedShapeFromPreviewMesh(
        const Urbaxio::CadKernel::MeshBuffers& previewMesh,
        const std::vector<unsigned int>& movingVertexIndices,
        const gp_Vec& translation,
        bool& changed) {
        changed = false;
        if (previewMesh.vertices.empty() || previewMesh.indices.empty() || movingVertexIndices.empty()) {
            return TopoDS_Shape();
        }

        const size_t vertexCount = previewMesh.vertices.size() / 3;
        if (vertexCount == 0) {
            return TopoDS_Shape();
        }

        std::unordered_set<unsigned int> movingSet;
        movingSet.reserve(movingVertexIndices.size());
        for (unsigned int index : movingVertexIndices) {
            if (index < vertexCount) {
                movingSet.insert(index);
            }
        }
        if (movingSet.empty()) {
            return TopoDS_Shape();
        }

        std::vector<gp_Pnt> movedPoints;
        movedPoints.reserve(vertexCount);
        for (size_t i = 0; i < vertexCount; ++i) {
            gp_Pnt point(
                previewMesh.vertices[i * 3 + 0],
                previewMesh.vertices[i * 3 + 1],
                previewMesh.vertices[i * 3 + 2]);
            if (movingSet.find(static_cast<unsigned int>(i)) != movingSet.end()) {
                point = TranslatedPoint(point, translation);
                changed = true;
            }
            movedPoints.push_back(point);
        }

        if (!changed) {
            return TopoDS_Shape();
        }

        TopoDS_Compound compound;
        BRep_Builder builder;
        builder.MakeCompound(compound);

        bool addedAny = false;
        for (size_t i = 0; i + 2 < previewMesh.indices.size(); i += 3) {
            const unsigned int ia = previewMesh.indices[i + 0];
            const unsigned int ib = previewMesh.indices[i + 1];
            const unsigned int ic = previewMesh.indices[i + 2];
            if (ia >= movedPoints.size() || ib >= movedPoints.size() || ic >= movedPoints.size()) {
                continue;
            }

            TopoDS_Face triangle = BuildTriangleFace(
                movedPoints[ia],
                movedPoints[ib],
                movedPoints[ic],
                TopAbs_FORWARD);
            if (triangle.IsNull()) {
                continue;
            }

            builder.Add(compound, triangle);
            addedAny = true;
        }

        return addedAny ? TopoDS_Shape(compound) : TopoDS_Shape();
    }

    static TopoDS_Shape BuildMovedFace(
        const TopoDS_Face& originalFace,
        const TopTools_IndexedMapOfShape& selectedVertices,
        const std::vector<glm::vec3>& selectedPositions,
        const gp_Vec& translation,
        const double tolerance,
        bool& changed) {
        changed = false;

        std::vector<TopoDS_Wire> movedWires;
        std::vector<std::vector<TopoDS_Edge>> movedWireEdges;
        std::vector<gp_Pnt> outerPoints;

        int wireIndex = 0;
        for (TopExp_Explorer wireExp(originalFace, TopAbs_WIRE); wireExp.More(); wireExp.Next(), ++wireIndex) {
            TopoDS_Wire movedWire;
            std::vector<TopoDS_Edge> movedEdges;
            std::vector<gp_Pnt> orderedPoints;
            bool wireChanged = false;
            if (!BuildMovedWire(TopoDS::Wire(wireExp.Current()), originalFace, selectedVertices, selectedPositions, translation, tolerance,
                                movedWire, movedEdges, orderedPoints, wireChanged)) {
                return TopoDS_Shape();
            }

            changed = changed || wireChanged;
            if (wireIndex == 0) {
                outerPoints = orderedPoints;
            }
            movedWires.push_back(movedWire);
            movedWireEdges.push_back(std::move(movedEdges));
        }

        if (!changed) {
            return originalFace;
        }
        if (movedWires.empty()) {
            return TopoDS_Shape();
        }

        gp_Pln plane;
        const double planarTolerance = std::max(tolerance * 10.0, 1.0e-6);
        if (TryMakePlaneFromPoints(outerPoints, planarTolerance, plane)) {
            BRepBuilderAPI_MakeFace faceMaker(plane, movedWires.front(), Standard_True);
            if (!faceMaker.IsDone()) {
                bool triangulatedChanged = false;
                return BuildTriangulatedMovedFaceFromMesh(originalFace, selectedVertices, selectedPositions, translation, tolerance, triangulatedChanged);
            }
            for (size_t i = 1; i < movedWires.size(); ++i) {
                faceMaker.Add(movedWires[i]);
            }
            if (!faceMaker.IsDone()) {
                bool triangulatedChanged = false;
                return BuildTriangulatedMovedFaceFromMesh(originalFace, selectedVertices, selectedPositions, translation, tolerance, triangulatedChanged);
            }
            TopoDS_Face movedFace = faceMaker.Face();
            movedFace.Orientation(originalFace.Orientation());
            return movedFace;
        }

        if (movedWires.size() > 1) {
            bool triangulatedChanged = false;
            TopoDS_Shape triangulated = BuildTriangulatedMovedFaceFromMesh(originalFace, selectedVertices, selectedPositions, translation, tolerance, triangulatedChanged);
            if (!triangulated.IsNull()) {
                return triangulated;
            }
            return TopoDS_Shape();
        }

        return BuildTriangulatedFaceCompound(outerPoints, originalFace.Orientation());
    }

    static bool ShapeContainsSolid(const TopoDS_Shape& shape) {
        if (shape.ShapeType() == TopAbs_SOLID || shape.ShapeType() == TopAbs_COMPSOLID) {
            return true;
        }
        for (TopExp_Explorer solidExp(shape, TopAbs_SOLID); solidExp.More(); solidExp.Next()) {
            return true;
        }
        return false;
    }

    static int CountFaces(const TopoDS_Shape& shape) {
        int faceCount = 0;
        for (TopExp_Explorer faceExp(shape, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
            ++faceCount;
        }
        return faceCount;
    }

    static TopoDS_Shape OrientClosedSolids(const TopoDS_Shape& shape) {
        if (shape.IsNull()) {
            return shape;
        }

        try {
            if (shape.ShapeType() == TopAbs_SOLID) {
                TopoDS_Solid solid = TopoDS::Solid(shape);
                BRepLib::OrientClosedSolid(solid);
                return solid;
            }

            Handle(ShapeBuild_ReShape) reshaper = new ShapeBuild_ReShape();
            bool modified = false;

            for (TopExp_Explorer solidExp(shape, TopAbs_SOLID); solidExp.More(); solidExp.Next()) {
                TopoDS_Solid solid = TopoDS::Solid(solidExp.Current());
                BRepLib::OrientClosedSolid(solid);
                reshaper->Replace(solidExp.Current(), solid);
                modified = true;
            }

            return modified ? reshaper->Apply(shape) : shape;
        } catch (const Standard_Failure& e) {
            std::cerr << "Warning: failed to orient closed solid: " << e.GetMessageString() << std::endl;
            return shape;
        }
    }

    static bool ShouldPreferPreviewTopologyMove(
        const TopoDS_Shape& originalShape,
        const Urbaxio::CadKernel::MeshBuffers& previewMesh) {
        const int faceCount = CountFaces(originalShape);
        const size_t triangleCount = previewMesh.indices.size() / 3;
        return faceCount > 12 || triangleCount > 24;
    }

    static TopoDS_Shape HealMovedShape(const TopoDS_Shape& candidate, const TopoDS_Shape& originalShape, const double tolerance) {
        TopoDS_Shape result = candidate;
        const double precision = std::max(1.0e-7, tolerance * 0.1);
        const double maxTolerance = std::max(1.0e-5, tolerance * 20.0);

        try {
            BRepLib::BuildCurves3d(result);
            BRepLib::SameParameter(result, precision, Standard_True);
        } catch (const Standard_Failure& e) {
            std::cerr << "MoveSubObject Warning: curve rebuild failed: " << e.GetMessageString() << std::endl;
        }

        try {
            ShapeFix_Shape fixer(result);
            fixer.SetPrecision(precision);
            fixer.SetMaxTolerance(maxTolerance);
            fixer.Perform();
            result = fixer.Shape();
        } catch (const Standard_Failure& e) {
            std::cerr << "MoveSubObject Warning: shape fix failed: " << e.GetMessageString() << std::endl;
        }

        const bool originalIsSolid = ShapeContainsSolid(originalShape);
        if (!originalIsSolid && BRepCheck_Analyzer(result).IsValid()) {
            return result;
        }

        try {
            BRepBuilderAPI_Sewing sewing(maxTolerance, Standard_True, Standard_True, Standard_True, Standard_True);
            sewing.SetNonManifoldMode(Standard_False);
            sewing.Add(result);
            sewing.Perform();
            TopoDS_Shape sewed = sewing.SewedShape();
            if (!sewed.IsNull()) {
                result = sewed;
            }
        } catch (const Standard_Failure& e) {
            std::cerr << "MoveSubObject Warning: sewing failed: " << e.GetMessageString() << std::endl;
        }

        if (originalIsSolid) {
            try {
                TopoDS_Compound compound;
                BRep_Builder builder;
                builder.MakeCompound(compound);
                bool madeSolid = false;

                for (TopExp_Explorer shellExp(result, TopAbs_SHELL); shellExp.More(); shellExp.Next()) {
                    BRepBuilderAPI_MakeSolid solidMaker(TopoDS::Shell(shellExp.Current()));
                    if (solidMaker.IsDone()) {
                        TopoDS_Solid solid = solidMaker.Solid();
                        BRepLib::OrientClosedSolid(solid);
                        builder.Add(compound, solid);
                        madeSolid = true;
                    }
                }

                if (!madeSolid) {
                    TopoDS_Shell rebuiltShell;
                    builder.MakeShell(rebuiltShell);
                    for (TopExp_Explorer faceExp(result, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
                        builder.Add(rebuiltShell, TopoDS::Face(faceExp.Current()));
                    }

                    try {
                        ShapeFix_Shell shellFixer;
                        shellFixer.Init(rebuiltShell);
                        shellFixer.SetPrecision(precision);
                        shellFixer.Perform();
                        TopoDS_Shell fixedShell = shellFixer.Shell();
                        if (!fixedShell.IsNull()) {
                            rebuiltShell = fixedShell;
                        }
                    } catch (const Standard_Failure& e) {
                        std::cerr << "MoveSubObject Warning: shell orientation fix failed: " << e.GetMessageString() << std::endl;
                    }

                    BRepBuilderAPI_MakeSolid solidMaker(rebuiltShell);
                    if (solidMaker.IsDone()) {
                        TopoDS_Solid solid = solidMaker.Solid();
                        BRepLib::OrientClosedSolid(solid);
                        builder.Add(compound, solid);
                        madeSolid = true;
                    }
                }

                if (madeSolid) {
                    result = compound;
                    if (BRepCheck_Analyzer(result).IsValid()) {
                        return result;
                    }
                }
            } catch (const Standard_Failure& e) {
                std::cerr << "MoveSubObject Warning: solid rebuild failed: " << e.GetMessageString() << std::endl;
            }
        }

        try {
            ShapeFix_Shape finalFixer(result);
            finalFixer.SetPrecision(precision);
            finalFixer.SetMaxTolerance(maxTolerance);
            finalFixer.Perform();
            result = finalFixer.Shape();
        } catch (const Standard_Failure& e) {
            std::cerr << "MoveSubObject Warning: final shape fix failed: " << e.GetMessageString() << std::endl;
        }

        return result;
    }

    static TopoDS_Shape BuildMovedShapeFromFaces(
        const TopoDS_Shape& originalShape,
        const TopTools_IndexedMapOfShape& selectedVertices,
        const std::vector<glm::vec3>& selectedPositions,
        const gp_Vec& translation,
        const double tolerance,
        int& matchedFaceCount,
        int& changedFaceCount) {
        matchedFaceCount = 0;
        changedFaceCount = 0;

        TopoDS_Compound faceCompound;
        BRep_Builder builder;
        builder.MakeCompound(faceCompound);

        for (TopExp_Explorer faceExp(originalShape, TopAbs_FACE); faceExp.More(); faceExp.Next()) {
            TopoDS_Face originalFace = TopoDS::Face(faceExp.Current());
            TopoDS_Shape faceToAdd = originalFace;

            if (FaceUsesAnySelectedVertex(originalFace, selectedVertices, selectedPositions, tolerance)) {
                ++matchedFaceCount;
                bool faceChanged = false;
                faceToAdd = BuildMovedFace(originalFace, selectedVertices, selectedPositions, translation, tolerance, faceChanged);
                if (faceToAdd.IsNull()) {
                    return TopoDS_Shape();
                }
                if (faceChanged) {
                    ++changedFaceCount;
                }
            }

            builder.Add(faceCompound, faceToAdd);
        }

        return faceCompound;
    }

    const float COPLANARITY_TOLERANCE_SCENE = 1e-4f;
    const int MAX_DFS_DEPTH = 50; // Max recursion depth for DFS to prevent stack overflow

    bool AreVec3Equal(const glm::vec3& a, const glm::vec3& b) {
        return glm::all(glm::epsilonEqual(a, b, Urbaxio::SCENE_POINT_EQUALITY_TOLERANCE));
    }

    struct FaceCutCandidate {
        uint64_t lineId = 0;
        Line line;
        int startVertex = -1;
        int endVertex = -1;
    };

    static int FindOrAddGraphVertex(std::vector<glm::vec3>& vertices, const glm::vec3& point) {
        for (size_t i = 0; i < vertices.size(); ++i) {
            if (AreVec3Equal(vertices[i], point)) {
                return static_cast<int>(i);
            }
        }
        vertices.push_back(point);
        return static_cast<int>(vertices.size() - 1);
    }

    static std::set<uint64_t> SelectSplittingLineIdsForFace(
        const TopoDS_Face& face,
        const std::vector<std::pair<uint64_t, Line>>& candidateLines,
        const double baseTolerance) {
        std::set<uint64_t> selectedIds;
        if (face.IsNull() || candidateLines.empty()) {
            return selectedIds;
        }

        std::vector<glm::vec3> vertices;
        std::vector<FaceCutCandidate> candidates;
        candidates.reserve(candidateLines.size());

        for (const auto& [lineId, line] : candidateLines) {
            if (LineLiesOnFaceBoundary(face, line, baseTolerance)) {
                continue;
            }

            FaceCutCandidate candidate;
            candidate.lineId = lineId;
            candidate.line = line;
            candidate.startVertex = FindOrAddGraphVertex(vertices, line.start);
            candidate.endVertex = FindOrAddGraphVertex(vertices, line.end);
            if (candidate.startVertex == candidate.endVertex) {
                continue;
            }
            candidates.push_back(candidate);
        }

        if (candidates.empty()) {
            return selectedIds;
        }

        std::vector<std::vector<int>> vertexToLines(vertices.size());
        for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
            vertexToLines[candidates[i].startVertex].push_back(i);
            vertexToLines[candidates[i].endVertex].push_back(i);
        }

        std::vector<bool> vertexOnBoundary(vertices.size(), false);
        for (size_t i = 0; i < vertices.size(); ++i) {
            vertexOnBoundary[i] = PointLiesOnFaceBoundary(
                face,
                gp_Pnt(vertices[i].x, vertices[i].y, vertices[i].z),
                baseTolerance);
        }

        std::vector<bool> visited(candidates.size(), false);
        for (int startLine = 0; startLine < static_cast<int>(candidates.size()); ++startLine) {
            if (visited[startLine]) {
                continue;
            }

            std::vector<int> componentLines;
            std::set<int> componentVertices;
            std::deque<int> queue;
            queue.push_back(startLine);
            visited[startLine] = true;

            while (!queue.empty()) {
                const int lineIndex = queue.front();
                queue.pop_front();
                componentLines.push_back(lineIndex);

                const int a = candidates[lineIndex].startVertex;
                const int b = candidates[lineIndex].endVertex;
                componentVertices.insert(a);
                componentVertices.insert(b);

                for (const int vertexIndex : {a, b}) {
                    for (int neighborLine : vertexToLines[vertexIndex]) {
                        if (!visited[neighborLine]) {
                            visited[neighborLine] = true;
                            queue.push_back(neighborLine);
                        }
                    }
                }
            }

            std::map<int, int> componentDegree;
            for (int lineIndex : componentLines) {
                ++componentDegree[candidates[lineIndex].startVertex];
                ++componentDegree[candidates[lineIndex].endVertex];
            }

            bool hasDanglingInteriorEnd = false;
            for (const int vertexIndex : componentVertices) {
                const int degree = componentDegree[vertexIndex];
                if (degree <= 1 && !vertexOnBoundary[vertexIndex]) {
                    hasDanglingInteriorEnd = true;
                    break;
                }
            }

            if (hasDanglingInteriorEnd) {
                continue;
            }

            for (int lineIndex : componentLines) {
                selectedIds.insert(candidates[lineIndex].lineId);
            }
        }

        return selectedIds;
    }

    static bool HasNonCollinearTriple(const std::vector<glm::vec3>& points) {
        if (points.size() < 3) {
            return false;
        }

        for (size_t i = 0; i < points.size(); ++i) {
            for (size_t j = i + 1; j < points.size(); ++j) {
                const glm::vec3 ij = points[j] - points[i];
                if (glm::length2(ij) < SCENE_POINT_EQUALITY_TOLERANCE * SCENE_POINT_EQUALITY_TOLERANCE) {
                    continue;
                }
                for (size_t k = j + 1; k < points.size(); ++k) {
                    const glm::vec3 ik = points[k] - points[i];
                    if (glm::length2(glm::cross(ij, ik)) >
                        SCENE_POINT_EQUALITY_TOLERANCE * SCENE_POINT_EQUALITY_TOLERANCE) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    static double PolygonArea3D(const std::vector<glm::vec3>& vertices) {
        if (vertices.size() < 3) {
            return 0.0;
        }

        glm::vec3 areaVector(0.0f);
        for (size_t i = 0; i < vertices.size(); ++i) {
            const glm::vec3& a = vertices[i];
            const glm::vec3& b = vertices[(i + 1) % vertices.size()];
            areaVector += glm::cross(a, b);
        }
        return static_cast<double>(glm::length(areaVector)) * 0.5;
    }

    static double PolygonPerimeter(const std::vector<glm::vec3>& vertices) {
        if (vertices.size() < 2) {
            return 0.0;
        }

        double perimeter = 0.0;
        for (size_t i = 0; i < vertices.size(); ++i) {
            perimeter += static_cast<double>(glm::distance(vertices[i], vertices[(i + 1) % vertices.size()]));
        }
        return perimeter;
    }

    Scene::Scene() {
        commandManager_ = std::make_unique<CommandManager>();
        materialManager_ = std::make_unique<MaterialManager>();
        meshManager_ = std::make_unique<MeshManager>();
        isStaticGeometryDirty_ = true; // --- NEW: Start with a dirty flag
    }
    Scene::~Scene() = default;

    CommandManager* Scene::getCommandManager() {
        return commandManager_.get();
    }

    void Scene::setGeometryMode(GeometryMode mode) {
        geometryMode_ = mode;
    }

    GeometryMode Scene::getGeometryMode() const {
        return geometryMode_;
    }

    MaterialManager* Scene::getMaterialManager() {
        return materialManager_.get();
    }

    MeshManager* Scene::getMeshManager() {
        return meshManager_.get();
    }

    const MeshManager* Scene::getMeshManager() const {
        return meshManager_.get();
    }

    // --- NEW: Scene Memento Implementation ---
    std::unique_ptr<SceneState> Scene::CaptureState() {
        auto state = std::make_unique<SceneState>();
        
        state->lines = this->lines_;
        state->vertexAdjacency = this->vertexAdjacency_;
        state->nextObjectId = this->next_object_id_;
        state->nextLineId = this->next_line_id_;
        
        for (const auto& [id, obj_ptr] : this->objects_) {
            // --- MODIFICATION: Only capture exportable objects ---
            if (!obj_ptr->isExportable()) {
                continue;
            }

            ObjectState objState;
            objState.id = id;
            objState.name = obj_ptr->get_name();

            // Serialize geometry based on type
            if (auto* meshGeom = dynamic_cast<MeshGeometry*>(obj_ptr->getGeometry())) {
                objState.isMeshGeometry = true;
                objState.serializedMesh = meshGeom->serialize();
            } else if (auto* brepGeom = dynamic_cast<BRepGeometry*>(obj_ptr->getGeometry())) {
                objState.isMeshGeometry = false;
                const TopoDS_Shape* shape = brepGeom->getShape();
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

        // --- MODIFICATION: Remove only EXPORTABLE objects that are not in the new state ---
        for (auto it = this->objects_.begin(); it != this->objects_.end(); ) {
            bool isExportable = it->second->isExportable();
            bool isInNewState = (newStateObjectIds.find(it->first) != newStateObjectIds.end());
            
            if (isExportable && !isInNewState) {
                it = this->objects_.erase(it);
            } else {
                ++it;
            }
        }
        
        // Update or create objects from the state (this part is correct as it only touches exportable objects)
        for (const auto& [id, objState] : state.objects) {
            SceneObject* obj = this->get_object_by_id(id);
            if (!obj) {
                obj = this->create_object_with_id(id, objState.name);
                obj->setExportable(true); // Ensure newly created objects from state are exportable
            }

            // Restore geometry based on type
            if (objState.isMeshGeometry && !objState.serializedMesh.empty()) {
                auto meshGeom = MeshGeometry::deserialize(objState.serializedMesh);
                if (meshGeom) {
                    obj->setGeometry(std::move(meshGeom));
                    this->UpdateObjectBoundary(obj);
                }
            } else if (!objState.serializedShape.empty()) {
                TopoDS_Shape restoredShape;
                std::stringstream ss(std::string(objState.serializedShape.begin(), objState.serializedShape.end()));
                try {
                    BinTools::Read(restoredShape, ss);
                    auto brepGeom = std::make_unique<BRepGeometry>(CadKernel::OCCT_ShapeUniquePtr(new TopoDS_Shape(restoredShape)));
                    obj->setGeometry(std::move(brepGeom));
                    this->UpdateObjectBoundary(obj);
                } catch(...) { /* error */ }
            } else if (obj->getGeometry()) {
                // If the object in the state has no geometry data, remove geometry from the live object.
                obj->setGeometry(nullptr);
            }
        }
        MarkStaticGeometryDirty();
    }

    SceneObject* Scene::create_object(const std::string& name) { 
        uint64_t new_id = next_object_id_++; 
        auto result = objects_.emplace(new_id, std::make_unique<SceneObject>(new_id, name)); 
        if (result.second) { 
            MarkStaticGeometryDirty(); // --- NEW: Mark as dirty when creating any object
            return result.first->second.get(); 
        } else { 
            std::cerr << "Scene: Failed to insert new object with ID " << new_id << " into map." << std::endl; 
            next_object_id_--; 
            return nullptr; 
        } 
    }

SceneObject* Scene::create_object_with_id(uint64_t id, const std::string& name) {
    // Check if ID is already taken
    if (objects_.count(id)) {
        return nullptr;
    }
    auto result = objects_.emplace(id, std::make_unique<SceneObject>(id, name));
    if (result.second) {
        // Ensure next generated ID will be higher than any loaded ID
        next_object_id_ = std::max(next_object_id_, id + 1);
        MarkStaticGeometryDirty(); // --- NEW: Mark as dirty
        return result.first->second.get();
    }
    return nullptr;
}

    SceneObject* Scene::create_box_object(const std::string& name, double dx, double dy, double dz) {
        SceneObject* new_obj = create_object(name);
        if (!new_obj) {
            return nullptr;
        }
        
        Urbaxio::CadKernel::OCCT_ShapeUniquePtr box_shape_ptr = Urbaxio::CadKernel::create_box(dx, dy, dz);
        if (!box_shape_ptr || box_shape_ptr->IsNull()) {
            // If creation fails, we should remove the empty object to avoid clutter.
            DeleteObject(new_obj->get_id());
            return nullptr;
        }

        // NEW: Wrap the shape in our BRepGeometry class
        auto geometry = std::make_unique<BRepGeometry>(std::move(box_shape_ptr));
        
        // NEW: Set the polymorphic geometry on the object
        new_obj->setGeometry(std::move(geometry));

        // The rest of the logic remains the same
        UpdateObjectBoundary(new_obj);
        new_obj->setExportable(true);
        MarkStaticGeometryDirty();
        
        // Note: Triangulation is now lazy, it will happen when getMeshBuffers() is first called.
        return new_obj;
    }
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
        
        // Clear geometry which will also invalidate mesh cache
        obj->setGeometry(nullptr);

        // Erase the object from the map, which will call its destructor and free memory
        objects_.erase(it);
        MarkStaticGeometryDirty();
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
            std::map<uint64_t, std::vector<glm::vec3>> splits_to_perform;
            std::vector<glm::vec3> split_points_on_new_line;
            for (const auto& [line_id, line] : lines_) {
                if (PointOnLineSegment(start, line.start, line.end)) {
                    splits_to_perform[line_id].push_back(start);
                }
                if (PointOnLineSegment(end, line.start, line.end)) {
                    splits_to_perform[line_id].push_back(end);
                }
                if (PointOnLineSegment(line.start, start, end)) {
                    split_points_on_new_line.push_back(line.start);
                }
                if (PointOnLineSegment(line.end, start, end)) {
                    split_points_on_new_line.push_back(line.end);
                }
            }
            for(const auto& split : splits_to_perform) {
                SplitLineAtPoints(split.first, split.second);
            }

            // 2. Find T-junction intersections between the new line segment and existing lines.
            std::map<uint64_t, std::vector<glm::vec3>> intersections_on_existing_lines;

            for (const auto& [existing_id, existing_line] : lines_) {
                glm::vec3 intersection_point;
                if (LineSegmentIntersection(start, end, existing_line.start, existing_line.end, intersection_point)) {
                    intersections_on_existing_lines[existing_id].push_back(intersection_point);
                    split_points_on_new_line.push_back(intersection_point);
                }
            }
            for (const auto& [line_id, points] : intersections_on_existing_lines) {
                SplitLineAtPoints(line_id, points);
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
                uint64_t new_id = AddSingleLineSegment(all_points[i], all_points[i+1], true);
                if (new_id != 0) {
                    new_line_ids.push_back(new_id);
                }
            }
            
            const bool splitExistingBRep = TrySplitBRepFacesWithLineGraph(new_line_ids);
            if (!splitExistingBRep) {
                for (uint64_t id : new_line_ids) {
                    FindAndCreateFaces(id);
                }
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
    uint64_t Scene::AddSingleLineSegment(const glm::vec3& start, const glm::vec3& end, bool userDrawn) {
        glm::vec3 canonicalStart = MergeOrAddVertex(start);
        glm::vec3 canonicalEnd = MergeOrAddVertex(end);

        if (AreVec3Equal(canonicalStart, canonicalEnd)) {
            return 0; // Invalid line
        }

        // Check for duplicate lines
        for(const auto& [id, line] : lines_) {
            if ((AreVec3Equal(line.start, canonicalStart) && AreVec3Equal(line.end, canonicalEnd)) ||
                (AreVec3Equal(line.start, canonicalEnd) && AreVec3Equal(line.end, canonicalStart))) {
                if (userDrawn) {
                    lines_[id].isUserDrawn = true;
                }
                return id; // Duplicate line, return existing ID
            }
        }
        
        uint64_t newLineId = next_line_id_++;
        lines_[newLineId] = {canonicalStart, canonicalEnd, false, userDrawn};

        vertexAdjacency_[canonicalStart].push_back(newLineId);
        vertexAdjacency_[canonicalEnd].push_back(newLineId);
        
        return newLineId;
    }

    bool Scene::HasLineSegment(const glm::vec3& start, const glm::vec3& end) const {
        for (const auto& [id, line] : lines_) {
            if ((AreVec3Equal(line.start, start) && AreVec3Equal(line.end, end)) ||
                (AreVec3Equal(line.start, end) && AreVec3Equal(line.end, start))) {
                return true;
            }
        }
        return false;
    }

    std::vector<PlanarLineSegmentSnapshot> Scene::CaptureObjectPlanarLineGraph(const SceneObject* obj) const {
        std::vector<PlanarLineSegmentSnapshot> segments;
        if (!obj) {
            return segments;
        }

        auto* brepGeom = dynamic_cast<BRepGeometry*>(obj->getGeometry());
        if (!brepGeom || !brepGeom->getShape() || brepGeom->getShape()->IsNull()) {
            return segments;
        }

        const TopoDS_Shape& shape = *brepGeom->getShape();
        const double matchTolerance = ShapeMatchTolerance(shape);

        auto addUniqueSegment = [&](const glm::vec3& a, const glm::vec3& b, const glm::vec3& normal, const float offset) {
            if (AreVec3Equal(a, b)) {
                return;
            }
            for (const auto& existing : segments) {
                const bool sameSegment =
                    (AreVec3Equal(existing.start, a) && AreVec3Equal(existing.end, b)) ||
                    (AreVec3Equal(existing.start, b) && AreVec3Equal(existing.end, a));
                const bool samePlane =
                    std::abs(glm::dot(glm::normalize(existing.planeNormal), glm::normalize(normal))) >
                    std::cos(1.0e-4f) &&
                    std::abs(existing.planeOffset - offset) <= 1.0e-4f;
                if (sameSegment && samePlane) {
                    return;
                }
            }
            segments.push_back({a, b, normal, offset});
        };

        for (const auto& [lineId, line] : lines_) {
            if (!line.isUserDrawn) {
                continue;
            }

            TopoDS_Face hostFace = FindPlanarFaceContainingLine(shape, line, matchTolerance);
            if (hostFace.IsNull()) {
                continue;
            }

            try {
                BRepAdaptor_Surface adaptor(hostFace, Standard_False);
                if (adaptor.GetType() != GeomAbs_Plane) {
                    continue;
                }

                const gp_Dir dir = adaptor.Plane().Axis().Direction();
                glm::vec3 normal(
                    static_cast<float>(dir.X()),
                    static_cast<float>(dir.Y()),
                    static_cast<float>(dir.Z()));
                if (glm::length2(normal) <= 1.0e-12f) {
                    continue;
                }
                normal = glm::normalize(normal);
                const float offset = glm::dot(normal, line.start);
                addUniqueSegment(line.start, line.end, normal, offset);
            } catch (const Standard_Failure&) {
            }
        }

        return segments;
    }

    void Scene::RestoreObjectPlanarLineGraph(
        SceneObject* obj,
        const std::vector<PlanarLineSegmentSnapshot>& segments) {
        if (!obj || segments.empty()) {
            return;
        }

        auto* brepGeom = dynamic_cast<BRepGeometry*>(obj->getGeometry());
        if (!brepGeom || !brepGeom->getShape() || brepGeom->getShape()->IsNull()) {
            return;
        }

        const TopoDS_Shape& shape = *brepGeom->getShape();
        const double matchTolerance = ShapeMatchTolerance(shape);
        std::vector<uint64_t> restoredLineIds;
        std::vector<std::pair<glm::vec3, glm::vec3>> clippedSegments;
        std::set<uint64_t> originalLineIdsToRemove;

        auto findLineSegmentId = [this](const glm::vec3& start, const glm::vec3& end) -> uint64_t {
            for (const auto& [id, line] : lines_) {
                if ((AreVec3Equal(line.start, start) && AreVec3Equal(line.end, end)) ||
                    (AreVec3Equal(line.start, end) && AreVec3Equal(line.end, start))) {
                    return id;
                }
            }
            return 0;
        };

        auto addUniqueT = [](std::vector<float>& values, const float t) {
            if (t < -1.0e-4f || t > 1.0f + 1.0e-4f) {
                return;
            }
            const float clamped = glm::clamp(t, 0.0f, 1.0f);
            for (float existing : values) {
                if (std::abs(existing - clamped) <= 1.0e-4f) {
                    return;
                }
            }
            values.push_back(clamped);
        };

        auto addUniqueClippedSegment = [&](const glm::vec3& a, const glm::vec3& b) {
            if (AreVec3Equal(a, b)) {
                return;
            }
            for (const auto& existing : clippedSegments) {
                if ((AreVec3Equal(existing.first, a) && AreVec3Equal(existing.second, b)) ||
                    (AreVec3Equal(existing.first, b) && AreVec3Equal(existing.second, a))) {
                    return;
                }
            }
            clippedSegments.push_back({a, b});
        };

        auto addUniquePoint = [](std::vector<glm::vec3>& points, const glm::vec3& point) {
            for (const glm::vec3& existing : points) {
                if (AreVec3Equal(existing, point)) {
                    return;
                }
            }
            points.push_back(point);
        };

        auto addNodedUserSegment = [&](const glm::vec3& start, const glm::vec3& end) {
            if (AreVec3Equal(start, end)) {
                return;
            }

            std::map<uint64_t, std::vector<glm::vec3>> splitsToPerform;
            std::vector<glm::vec3> splitPointsOnNewLine;

            std::vector<std::pair<uint64_t, Line>> lineSnapshot;
            lineSnapshot.reserve(lines_.size());
            for (const auto& [lineId, line] : lines_) {
                lineSnapshot.push_back({lineId, line});
            }

            for (const auto& [lineId, line] : lineSnapshot) {
                if (PointOnLineSegmentInclusive(start, line.start, line.end)) {
                    splitsToPerform[lineId].push_back(start);
                }
                if (PointOnLineSegmentInclusive(end, line.start, line.end)) {
                    splitsToPerform[lineId].push_back(end);
                }
                if (PointOnLineSegmentInclusive(line.start, start, end)) {
                    addUniquePoint(splitPointsOnNewLine, line.start);
                }
                if (PointOnLineSegmentInclusive(line.end, start, end)) {
                    addUniquePoint(splitPointsOnNewLine, line.end);
                }

                glm::vec3 intersection;
                if (LineSegmentIntersectionInclusive(start, end, line.start, line.end, intersection)) {
                    splitsToPerform[lineId].push_back(intersection);
                    addUniquePoint(splitPointsOnNewLine, intersection);
                }
            }

            for (const auto& [lineId, splitPoints] : splitsToPerform) {
                SplitLineAtPoints(lineId, splitPoints);
            }

            std::vector<glm::vec3> allPoints = {start};
            allPoints.insert(allPoints.end(), splitPointsOnNewLine.begin(), splitPointsOnNewLine.end());
            allPoints.push_back(end);

            const glm::vec3 direction = end - start;
            std::sort(allPoints.begin(), allPoints.end(), [&start, &direction](const glm::vec3& a, const glm::vec3& b) {
                if (glm::length2(direction) < 1.0e-9f) {
                    return false;
                }
                return glm::dot(a - start, direction) < glm::dot(b - start, direction);
            });
            allPoints.erase(std::unique(allPoints.begin(), allPoints.end(), AreVec3Equal), allPoints.end());

            for (size_t i = 0; i + 1 < allPoints.size(); ++i) {
                uint64_t lineId = AddSingleLineSegment(allPoints[i], allPoints[i + 1], true);
                if (lineId != 0) {
                    lines_[lineId].isUserDrawn = true;
                    restoredLineIds.push_back(lineId);
                }
            }
        };

        for (const PlanarLineSegmentSnapshot& segment : segments) {
            const uint64_t existingLineId = findLineSegmentId(segment.start, segment.end);
            if (existingLineId != 0) {
                originalLineIdsToRemove.insert(existingLineId);
            }

            const glm::vec3 direction = segment.end - segment.start;
            const float directionLenSq = glm::length2(direction);
            if (directionLenSq <= 1.0e-12f || glm::length2(segment.planeNormal) <= 1.0e-12f) {
                continue;
            }

            for (TopExp_Explorer faceExplorer(shape, TopAbs_FACE); faceExplorer.More(); faceExplorer.Next()) {
                const TopoDS_Face face = TopoDS::Face(faceExplorer.Current());
                if (!LineLiesOnFacePlane(
                        face,
                        segment.start,
                        segment.end,
                        segment.planeNormal,
                        segment.planeOffset,
                        matchTolerance)) {
                    continue;
                }

                std::vector<float> splitParameters = {0.0f, 1.0f};

                for (TopExp_Explorer edgeExplorer(face, TopAbs_EDGE); edgeExplorer.More(); edgeExplorer.Next()) {
                    const TopoDS_Edge edge = TopoDS::Edge(edgeExplorer.Current());
                    TopoDS_Vertex v1, v2;
                    TopExp::Vertices(edge, v1, v2, Standard_True);
                    if (v1.IsNull() || v2.IsNull()) {
                        continue;
                    }

                    const gp_Pnt p1 = BRep_Tool::Pnt(v1);
                    const gp_Pnt p2 = BRep_Tool::Pnt(v2);
                    const glm::vec3 edgeStart(
                        static_cast<float>(p1.X()),
                        static_cast<float>(p1.Y()),
                        static_cast<float>(p1.Z()));
                    const glm::vec3 edgeEnd(
                        static_cast<float>(p2.X()),
                        static_cast<float>(p2.Y()),
                        static_cast<float>(p2.Z()));

                    glm::vec3 intersection;
                    if (LineSegmentIntersectionInclusive(segment.start, segment.end, edgeStart, edgeEnd, intersection)) {
                        addUniqueT(splitParameters, glm::dot(intersection - segment.start, direction) / directionLenSq);
                    }

                    if (PointOnLineSegmentInclusive(edgeStart, segment.start, segment.end)) {
                        addUniqueT(splitParameters, glm::dot(edgeStart - segment.start, direction) / directionLenSq);
                    }
                    if (PointOnLineSegmentInclusive(edgeEnd, segment.start, segment.end)) {
                        addUniqueT(splitParameters, glm::dot(edgeEnd - segment.start, direction) / directionLenSq);
                    }
                }

                std::sort(splitParameters.begin(), splitParameters.end());
                for (size_t i = 0; i + 1 < splitParameters.size(); ++i) {
                    const float t0 = splitParameters[i];
                    const float t1 = splitParameters[i + 1];
                    if (t1 - t0 <= 1.0e-4f) {
                        continue;
                    }

                    const float tm = (t0 + t1) * 0.5f;
                    const glm::vec3 midpoint = segment.start + direction * tm;
                    if (!PointLiesOnFaceByUV(face, gp_Pnt(midpoint.x, midpoint.y, midpoint.z), matchTolerance)) {
                        continue;
                    }

                    const glm::vec3 clippedStart = segment.start + direction * t0;
                    const glm::vec3 clippedEnd = segment.start + direction * t1;
                    addUniqueClippedSegment(clippedStart, clippedEnd);
                }
            }
        }

        for (uint64_t lineId : originalLineIdsToRemove) {
            RemoveLine(lineId);
        }

        for (const auto& [start, end] : clippedSegments) {
            addNodedUserSegment(start, end);
        }

        if (!restoredLineIds.empty()) {
            std::cout << "Scene: Restoring planar line graph on object " << obj->get_id()
                      << " with " << restoredLineIds.size() << " segments." << std::endl;
            TrySplitBRepFacesWithLineGraph(restoredLineIds);
        }
    }

    bool Scene::TrySplitBRepFacesWithLineGraph(const std::vector<uint64_t>& seedLineIds) {
        if (seedLineIds.empty() || geometryMode_ != GeometryMode::BRep) {
            return false;
        }

        std::set<uint64_t> seedSet(seedLineIds.begin(), seedLineIds.end());
        bool anyObjectSplit = false;

        std::vector<SceneObject*> objectsSnapshot;
        objectsSnapshot.reserve(objects_.size());
        for (const auto& [id, obj] : objects_) {
            if (obj && dynamic_cast<BRepGeometry*>(obj->getGeometry())) {
                objectsSnapshot.push_back(obj.get());
            }
        }

        for (SceneObject* hostObject : objectsSnapshot) {
            if (!hostObject) {
                continue;
            }

            auto* brepGeom = dynamic_cast<BRepGeometry*>(hostObject->getGeometry());
            if (!brepGeom || !brepGeom->getShape() || brepGeom->getShape()->IsNull()) {
                continue;
            }

            const TopoDS_Shape& originalShape = *brepGeom->getShape();
            const double matchTolerance = ShapeMatchTolerance(originalShape);

            ShapeFix_Shape shapeFixer(originalShape);
            shapeFixer.SetPrecision(std::max(1.0e-7, matchTolerance * 0.1));
            shapeFixer.SetMaxTolerance(std::max(1.0e-5, matchTolerance * 20.0));
            shapeFixer.Perform();
            TopoDS_Shape workingShape = shapeFixer.Shape();
            if (workingShape.IsNull()) {
                continue;
            }

            TopTools_ListOfShape tools;
            std::vector<uint64_t> cuttingLineIds;
            std::set<uint64_t> cuttingLineIdSet;
            bool touchesSeedLine = false;

            for (TopExp_Explorer faceExplorer(workingShape, TopAbs_FACE); faceExplorer.More(); faceExplorer.Next()) {
                TopoDS_Face hostFace = TopoDS::Face(faceExplorer.Current());
                try {
                    BRepAdaptor_Surface surfaceAdaptor(hostFace, Standard_False);
                    if (surfaceAdaptor.GetType() != GeomAbs_Plane) {
                        continue;
                    }
                } catch (const Standard_Failure&) {
                    continue;
                }

                std::vector<std::pair<uint64_t, Line>> candidateLines;
                for (const auto& [lineId, line] : lines_) {
                    if (hostObject->boundaryLineIDs.count(lineId) > 0) {
                        continue;
                    }
                    if (line.usedInFace && seedSet.count(lineId) == 0 && !line.isUserDrawn) {
                        continue;
                    }
                    if (!LineLiesOnFaceByUV(hostFace, line, matchTolerance)) {
                        continue;
                    }
                    candidateLines.push_back({lineId, line});
                }

                const std::set<uint64_t> selectedFaceLineIds =
                    SelectSplittingLineIdsForFace(hostFace, candidateLines, matchTolerance);

                for (uint64_t lineId : selectedFaceLineIds) {
                    const auto lineIt = lines_.find(lineId);
                    if (lineIt == lines_.end()) {
                        continue;
                    }

                    TopoDS_Edge cuttingEdge;
                    if (!MakeCuttingEdgeOnFace(hostFace, lineIt->second, matchTolerance, cuttingEdge)) {
                        continue;
                    }

                    tools.Append(cuttingEdge);
                    cuttingLineIds.push_back(lineId);
                    cuttingLineIdSet.insert(lineId);
                    touchesSeedLine = touchesSeedLine || seedSet.count(lineId) > 0;
                }
            }

            if (tools.IsEmpty() || !touchesSeedLine) {
                continue;
            }

            try {
                BOPAlgo_Splitter splitter;
                TopTools_ListOfShape arguments;
                arguments.Append(workingShape);
                splitter.SetArguments(arguments);
                splitter.SetTools(tools);

                const int faceCountBefore = CountFaces(workingShape);
                splitter.Perform();
                if (splitter.HasErrors()) {
                    splitter.GetReport()->Dump(std::cout);
                    continue;
                }

                TopoDS_Shape splitShape = splitter.Shape();
                if (splitShape.IsNull()) {
                    continue;
                }

                ShapeFix_Shape finalFixer(splitShape);
                finalFixer.SetPrecision(std::max(1.0e-7, matchTolerance * 0.1));
                finalFixer.SetMaxTolerance(std::max(1.0e-5, matchTolerance * 20.0));
                finalFixer.Perform();
                TopoDS_Shape finalShape = finalFixer.Shape();
                finalShape = OrientClosedSolids(finalShape);
                if (finalShape.IsNull()) {
                    continue;
                }

                const int faceCountAfter = CountFaces(finalShape);
                if (faceCountAfter <= faceCountBefore) {
                    continue;
                }

                BRepCheck_Analyzer analyzer(finalShape);
                if (!analyzer.IsValid()) {
                    std::cerr << "Line graph split produced an invalid B-Rep; keeping original shape." << std::endl;
                    continue;
                }

                brepGeom->setShape(CadKernel::OCCT_ShapeUniquePtr(new TopoDS_Shape(finalShape)));
                hostObject->invalidateMeshCache();
                UpdateObjectBoundary(hostObject);

                for (uint64_t lineId : cuttingLineIds) {
                    auto lineIt = lines_.find(lineId);
                    if (lineIt != lines_.end()) {
                        lineIt->second.usedInFace = true;
                    }
                }

                anyObjectSplit = true;
                std::cout << "Scene: Split object " << hostObject->get_id()
                          << " with planar line graph. faces "
                          << faceCountBefore << " -> " << faceCountAfter
                          << ", cuttingLines=" << cuttingLineIdSet.size() << std::endl;
            } catch (const Standard_Failure& e) {
                std::cerr << "OCCT Exception during line graph split: " << e.GetMessageString() << std::endl;
            }
        }

        if (anyObjectSplit) {
            MarkStaticGeometryDirty();
        }
        return anyObjectSplit;
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

    void Scene::SplitLineAtPoints(uint64_t lineId, const std::vector<glm::vec3>& splitPoints) {
        auto it = lines_.find(lineId);
        if (it == lines_.end() || splitPoints.empty()) {
            return;
        }

        Line originalLine = it->second;
        const glm::vec3 dir = originalLine.end - originalLine.start;
        if (glm::length2(dir) < 1.0e-12f) {
            return;
        }

        std::vector<glm::vec3> orderedPoints;
        orderedPoints.reserve(splitPoints.size() + 2);
        orderedPoints.push_back(originalLine.start);
        for (const glm::vec3& splitPoint : splitPoints) {
            if (PointOnLineSegment(splitPoint, originalLine.start, originalLine.end)) {
                orderedPoints.push_back(MergeOrAddVertex(splitPoint));
            }
        }
        orderedPoints.push_back(originalLine.end);

        std::sort(orderedPoints.begin(), orderedPoints.end(),
            [&originalLine, &dir](const glm::vec3& a, const glm::vec3& b) {
                return glm::dot(a - originalLine.start, dir) < glm::dot(b - originalLine.start, dir);
            });
        orderedPoints.erase(std::unique(orderedPoints.begin(), orderedPoints.end(), AreVec3Equal), orderedPoints.end());
        if (orderedPoints.size() <= 2) {
            return;
        }

        std::cout << "Scene: Splitting line " << lineId << " into "
                  << (orderedPoints.size() - 1) << " segments" << std::endl;

        RemoveLine(lineId);
        for (size_t i = 0; i + 1 < orderedPoints.size(); ++i) {
            uint64_t segmentId = AddSingleLineSegment(orderedPoints[i], orderedPoints[i + 1], originalLine.isUserDrawn);
            if (originalLine.usedInFace && segmentId != 0 && lines_.count(segmentId)) {
                lines_[segmentId].usedInFace = true;
            }
        }
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

        SplitLineAtPoints(lineId, {canonicalSplitPoint});
        
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

    bool Scene::FindBestCycleForLine(
        uint64_t newLineId,
        std::vector<glm::vec3>& orderedVertices,
        std::vector<uint64_t>& pathLineIDs) {
        auto lineIt = lines_.find(newLineId);
        if (lineIt == lines_.end() || lineIt->second.usedInFace) {
            return false;
        }

        struct CycleCandidate {
            std::vector<glm::vec3> vertices;
            std::vector<uint64_t> pathLineIDs;
            double area = 0.0;
            double perimeter = 0.0;
            int usedLineCount = 0;
        };

        const Line& newLine = lineIt->second;
        const glm::vec3 pA = newLine.start;
        const glm::vec3 pB = newLine.end;
        const double newLineLength = std::max(1.0e-6, static_cast<double>(glm::distance(pA, pB)));
        const size_t maxCandidates = 256;

        std::vector<CycleCandidate> candidates;
        std::vector<glm::vec3> currentPathVertices;
        std::vector<uint64_t> currentPathLineIDs;
        std::set<uint64_t> visitedLines;
        visitedLines.insert(newLineId);

        auto makeOrderedVertices = [&]() {
            std::vector<glm::vec3> vertices;
            vertices.reserve(currentPathVertices.size() + 2);
            vertices.push_back(pA);
            vertices.push_back(pB);
            if (!currentPathVertices.empty()) {
                for (size_t i = 0; i + 1 < currentPathVertices.size(); ++i) {
                    vertices.push_back(currentPathVertices[i]);
                }
            }
            return vertices;
        };

        auto loopHasSelfIntersection = [&](const std::vector<glm::vec3>& vertices) {
            if (vertices.size() < 4) {
                return false;
            }

            for (size_t i = 0; i < vertices.size(); ++i) {
                const glm::vec3 a1 = vertices[i];
                const glm::vec3 a2 = vertices[(i + 1) % vertices.size()];
                for (size_t j = i + 1; j < vertices.size(); ++j) {
                    if (j == i || j == i + 1) {
                        continue;
                    }
                    if (i == 0 && j + 1 == vertices.size()) {
                        continue;
                    }

                    const glm::vec3 b1 = vertices[j];
                    const glm::vec3 b2 = vertices[(j + 1) % vertices.size()];
                    glm::vec3 intersection;
                    if (LineSegmentIntersection(a1, a2, b1, b2, intersection)) {
                        return true;
                    }
                }
            }
            return false;
        };

        auto canRemainCoplanar = [&](const std::vector<glm::vec3>& vertices) {
            if (!HasNonCollinearTriple(vertices)) {
                return true;
            }

            gp_Pln tempPlane;
            return ArePointsCoplanar(vertices, tempPlane);
        };

        auto addCandidate = [&]() {
            std::vector<glm::vec3> vertices = makeOrderedVertices();
            if (vertices.size() < 3 || !HasNonCollinearTriple(vertices)) {
                return;
            }

            gp_Pln plane;
            if (!ArePointsCoplanar(vertices, plane) || loopHasSelfIntersection(vertices)) {
                return;
            }

            const double area = PolygonArea3D(vertices);
            const double perimeter = PolygonPerimeter(vertices);
            if (area <= 1.0e-10 || perimeter <= newLineLength) {
                return;
            }

            int usedLineCount = 0;
            for (uint64_t lineId : currentPathLineIDs) {
                auto pathLineIt = lines_.find(lineId);
                if (pathLineIt != lines_.end() && pathLineIt->second.usedInFace) {
                    ++usedLineCount;
                }
            }

            candidates.push_back({std::move(vertices), currentPathLineIDs, area, perimeter, usedLineCount});
        };

        std::function<void(const glm::vec3&, int, double)> search =
            [&](const glm::vec3& currentNode, int depth, double pathLength) {
                if (candidates.size() >= maxCandidates || depth > MAX_DFS_DEPTH) {
                    return;
                }

                if (AreVec3Equal(currentNode, pA)) {
                    if (currentPathVertices.size() >= 2) {
                        addCandidate();
                    }
                    return;
                }

                auto adjacencyIt = vertexAdjacency_.find(currentNode);
                if (adjacencyIt == vertexAdjacency_.end()) {
                    return;
                }

                std::vector<uint64_t> incidentLines = adjacencyIt->second;
                std::sort(incidentLines.begin(), incidentLines.end(), [&](uint64_t left, uint64_t right) {
                    const auto leftIt = lines_.find(left);
                    const auto rightIt = lines_.find(right);
                    if (leftIt == lines_.end() || rightIt == lines_.end()) {
                        return left < right;
                    }

                    if (leftIt->second.usedInFace != rightIt->second.usedInFace) {
                        return !leftIt->second.usedInFace;
                    }

                    const glm::vec3 leftOther = AreVec3Equal(leftIt->second.start, currentNode) ? leftIt->second.end : leftIt->second.start;
                    const glm::vec3 rightOther = AreVec3Equal(rightIt->second.start, currentNode) ? rightIt->second.end : rightIt->second.start;
                    const float leftLength = glm::distance2(leftOther, currentNode);
                    const float rightLength = glm::distance2(rightOther, currentNode);
                    if (std::abs(leftLength - rightLength) > 1.0e-8f) {
                        return leftLength < rightLength;
                    }
                    return left < right;
                });

                for (uint64_t lineId : incidentLines) {
                    if (lineId == newLineId || visitedLines.count(lineId)) {
                        continue;
                    }

                    auto candidateLineIt = lines_.find(lineId);
                    if (candidateLineIt == lines_.end()) {
                        continue;
                    }

                    const Line& candidateLine = candidateLineIt->second;
                    const glm::vec3 nextNode = AreVec3Equal(candidateLine.start, currentNode) ? candidateLine.end : candidateLine.start;
                    const double segmentLength = static_cast<double>(glm::distance(currentNode, nextNode));
                    if (segmentLength <= SCENE_POINT_EQUALITY_TOLERANCE) {
                        continue;
                    }

                    currentPathVertices.push_back(nextNode);
                    currentPathLineIDs.push_back(lineId);
                    visitedLines.insert(lineId);

                    std::vector<glm::vec3> tentativeVertices = makeOrderedVertices();
                    tentativeVertices.push_back(nextNode);

                    if (canRemainCoplanar(tentativeVertices)) {
                        search(nextNode, depth + 1, pathLength + segmentLength);
                    }

                    visitedLines.erase(lineId);
                    currentPathLineIDs.pop_back();
                    currentPathVertices.pop_back();
                }
            };

        search(pB, 0, 0.0);

        if (candidates.empty()) {
            return false;
        }

        std::sort(candidates.begin(), candidates.end(), [&](const CycleCandidate& left, const CycleCandidate& right) {
            const double areaTol = std::max(1.0e-8, std::min(left.area, right.area) * 1.0e-5);
            if (std::abs(left.area - right.area) > areaTol) {
                return left.area < right.area;
            }

            if (left.usedLineCount != right.usedLineCount) {
                return left.usedLineCount < right.usedLineCount;
            }

            const double perimeterTol = std::max(1.0e-8, std::min(left.perimeter, right.perimeter) * 1.0e-5);
            if (std::abs(left.perimeter - right.perimeter) > perimeterTol) {
                return left.perimeter < right.perimeter;
            }

            return left.vertices.size() < right.vertices.size();
        });

        orderedVertices = candidates.front().vertices;
        pathLineIDs = candidates.front().pathLineIDs;

        std::cout << "FindAndCreateFaces: Chose local loop for line " << newLineId
                  << " candidates=" << candidates.size()
                  << " vertices=" << orderedVertices.size()
                  << " area=" << candidates.front().area
                  << " perimeter=" << candidates.front().perimeter
                  << " usedLines=" << candidates.front().usedLineCount << std::endl;

        return true;
    }


    void Scene::CreateOCCTFace(const std::vector<glm::vec3>& orderedVertices, const gp_Pln& plane) {
        if (orderedVertices.size() < 3) return;

        // --- Mesh Mode: Create MeshGeometry face ---
        if (geometryMode_ == GeometryMode::Mesh) {
            gp_Dir normalDir = plane.Axis().Direction();
            glm::vec3 normal(static_cast<float>(normalDir.X()),
                             static_cast<float>(normalDir.Y()),
                             static_cast<float>(normalDir.Z()));

            auto meshGeom = MeshGeometry::createFromVertexLoop(orderedVertices, normal);
            if (meshGeom) {
                std::string face_name = "MeshFace_" + std::to_string(next_face_id_++);
                SceneObject* new_face_obj = create_object(face_name);
                if (new_face_obj) {
                    new_face_obj->setGeometry(std::move(meshGeom));
                    UpdateObjectBoundary(new_face_obj);
                    std::cout << "Scene: Auto-created Mesh Face object: " << face_name << std::endl;
                }
            }
            return;
        }

        // --- BRep Mode: Original OCCT logic ---
        BRepBuilderAPI_MakeWire wireMaker;
        try {
            for (size_t i = 0; i < orderedVertices.size(); ++i) {
                gp_Pnt p1_occt(orderedVertices[i].x, orderedVertices[i].y, orderedVertices[i].z);
                gp_Pnt p2_occt(orderedVertices[(i + 1) % orderedVertices.size()].x, orderedVertices[(i + 1) % orderedVertices.size()].y, orderedVertices[(i + 1) % orderedVertices.size()].z);
                if (p1_occt.IsEqual(p2_occt, SCENE_POINT_EQUALITY_TOLERANCE)) continue;
                wireMaker.Add(BRepBuilderAPI_MakeEdge(p1_occt, p2_occt));
            }
        } catch (const Standard_Failure& e) { std::cerr << "OCCT Exception during edge/vertex creation for face: " << e.GetMessageString() << std::endl; return; }

        if (wireMaker.IsDone() && !wireMaker.Wire().IsNull()) {
            TopoDS_Wire wire = wireMaker.Wire();
            BRepBuilderAPI_MakeFace faceMaker(plane, wire, Standard_True);
            if (faceMaker.IsDone() && !faceMaker.Face().IsNull()) {
                TopoDS_Face face = faceMaker.Face();
                std::string face_name = "AutoFace_" + std::to_string(next_face_id_++);
                SceneObject* new_face_obj = create_object(face_name);
                if (new_face_obj) {
                    // NEW: Use the new geometry system
                    auto geometry = std::make_unique<BRepGeometry>(CadKernel::OCCT_ShapeUniquePtr(new TopoDS_Shape(face)));
                    new_face_obj->setGeometry(std::move(geometry));
                    
                    UpdateObjectBoundary(new_face_obj); // <-- Sync lines
                    std::cout << "Scene: Auto-created Face object: " << face_name << std::endl;
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
        
        std::vector<uint64_t> pathLineIDsCollector;
        std::vector<glm::vec3> finalOrderedVertices;
        if (!FindBestCycleForLine(newLineId, finalOrderedVertices, pathLineIDsCollector)) {
            return;
        }

        if (finalOrderedVertices.size() < 3) return;

        SceneObject* hostObject = nullptr;
        TopoDS_Face originalHostFace;
        
        gp_Pln cyclePlane;
        const bool loopIsCoplanar = ArePointsCoplanar(finalOrderedVertices, cyclePlane);
        if (loopIsCoplanar) {
            gp_Dir normalDir = cyclePlane.Axis().Direction();
            glm::vec3 loopNormal(normalDir.X(), normalDir.Y(), normalDir.Z());
            
            for (const auto& [id, obj_ptr] : objects_) {
                if (obj_ptr && dynamic_cast<BRepGeometry*>(obj_ptr->getGeometry())) {
                    TopoDS_Face foundFace = FindOriginalFace(obj_ptr.get(), finalOrderedVertices, loopNormal);
                    if (!foundFace.IsNull()) {
                        hostObject = obj_ptr.get();
                        originalHostFace = foundFace;
                        break; 
                    }
                }
            }
        }

        if (!loopIsCoplanar) {
            std::cout << "FindAndCreateFaces: Loop detected but vertices are not coplanar; skipping face creation." << std::endl;
            return;
        }

        if (hostObject && !originalHostFace.IsNull()) {
            std::cout << "Scene: Detected new loop on existing face of object " << hostObject->get_id() << ". Attempting PARAMETRIC split." << std::endl;
            try {
                // NEW: Get shape from BRepGeometry
                auto* brepGeom = static_cast<BRepGeometry*>(hostObject->getGeometry());
                const TopoDS_Shape* hostShape = brepGeom->getShape();
                if (!hostShape) throw Standard_Failure("Host object has no B-Rep shape.");

                ShapeFix_Shape shapeFixer(*hostShape);
                shapeFixer.Perform();
                TopoDS_Shape healedShape = shapeFixer.Shape();
                AnalyzeShape(healedShape, "Healed Host Shape BEFORE split");
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
                BOPAlgo_Splitter splitter;
                TopTools_ListOfShape arguments;
                arguments.Append(healedShape);
                splitter.SetArguments(arguments);
                TopTools_ListOfShape tools;
                tools.Append(cuttingWire);
                splitter.SetTools(tools);
                TopExp_Explorer faceExpBefore(healedShape, TopAbs_FACE);
                int faceCountBefore = 0;
                for (; faceExpBefore.More(); faceExpBefore.Next()) faceCountBefore++;
                splitter.Perform();
                if (splitter.HasErrors()) {
                    splitter.GetReport()->Dump(std::cout);
                    throw Standard_Failure("BOPAlgo_Splitter failed to perform operation.");
                }
                TopoDS_Shape splitShape = splitter.Shape();
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
                
                // NEW: Update geometry through the new system
                brepGeom->setShape(CadKernel::OCCT_ShapeUniquePtr(new TopoDS_Shape(finalShape)));
                hostObject->invalidateMeshCache();
                UpdateObjectBoundary(hostObject);
                std::cout << "Scene: Object " << hostObject->get_id() << " was successfully split and updated." << std::endl;
            } catch (const Standard_Failure& e) {
                std::cerr << "OCCT Exception during parametric split: " << e.GetMessageString() << std::endl;
            }
        } else {
            std::cout << "FindAndCreateFaces: Loop detected, but no suitable host face found. Creating a new independent face object." << std::endl;
            CreateOCCTFace(finalOrderedVertices, cyclePlane);
            lines_.at(newLineId).usedInFace = true;
            for (uint64_t lineId : pathLineIDsCollector) {
                lines_.at(lineId).usedInFace = true;
            }
        }
    }
    
    // --- HELPER FUNCTIONS ---
    void Scene::UpdateObjectBoundary(SceneObject* obj) {
        if (!obj || !obj->hasGeometry()) return;
        
        // 1. Remove all old lines associated with this object.
        for (uint64_t oldLineId : obj->boundaryLineIDs) {
            auto lineIt = lines_.find(oldLineId);
            if (lineIt != lines_.end() && !lineIt->second.isUserDrawn) {
                RemoveLine(oldLineId);
            }
        }
        obj->boundaryLineIDs.clear();
        
        // 2. Extract fresh edges if it's a BRep shape.
        if (auto* brepGeom = dynamic_cast<BRepGeometry*>(obj->getGeometry())) {
            const TopoDS_Shape* shape = brepGeom->getShape();
            if (shape) {
                auto new_edges = ExtractEdgesFromBRepShape(*shape);
                
                // 3. Add the new edges as lines and update the object's boundary set.
                for (const auto& edge : new_edges) {
                    uint64_t line_id = AddSingleLineSegment(edge.first, edge.second);
                    if (line_id != 0) {
                        lines_[line_id].usedInFace = true;
                        if (!lines_[line_id].isUserDrawn) {
                            obj->boundaryLineIDs.insert(line_id);
                        }
                    }
                }
            }
        }
        // Note: For other geometry types like MeshGeometry or VolumetricGeometry,
        // this function does nothing, which is correct as they don't define B-Rep boundaries.
    }
    


    TopoDS_Face Scene::FindOriginalFace(SceneObject* obj, const std::vector<glm::vec3>& faceVertices, const glm::vec3& guideNormal) {
        if (faceVertices.empty() || !obj) return TopoDS_Face();
        
        // NEW: The shape now comes from the IGeometry interface
        auto* brepGeom = dynamic_cast<BRepGeometry*>(obj->getGeometry());
        if (!brepGeom) return TopoDS_Face();
        const TopoDS_Shape* shape = brepGeom->getShape();
        if (!shape) return TopoDS_Face();

        const double matchTolerance = ShapeMatchTolerance(*shape);
        TopTools_IndexedMapOfShape targetVertices = FindVerticesNearPositions(*shape, faceVertices, matchTolerance);

        std::vector<TopoDS_Face> candidates;
        if (targetVertices.Extent() > 0) {
            TopExp_Explorer faceExplorer(*shape, TopAbs_FACE);
            for (; faceExplorer.More(); faceExplorer.Next()) {
                TopoDS_Face candidateFace = TopoDS::Face(faceExplorer.Current());
                TopTools_IndexedMapOfShape faceVerticesSet;
                TopExp_Explorer vertexExplorer(candidateFace, TopAbs_VERTEX);
                for (; vertexExplorer.More(); vertexExplorer.Next()) {
                    faceVerticesSet.Add(TopoDS::Vertex(vertexExplorer.Current()));
                }

                bool contains_all_targets = true;
                for (int i = 1; i <= targetVertices.Extent(); ++i) {
                    if (!MapContainsSameShape(faceVerticesSet, targetVertices(i))) {
                        contains_all_targets = false;
                        break;
                    }
                }
                if (contains_all_targets) {
                    AddUniqueFaceCandidate(candidates, candidateFace);
                }
            }
        }

        TopExp_Explorer faceExplorer(*shape, TopAbs_FACE);
        for (; faceExplorer.More(); faceExplorer.Next()) {
            TopoDS_Face candidateFace = TopoDS::Face(faceExplorer.Current());
            try {
                BRepAdaptor_Surface surfaceAdaptor(candidateFace, Standard_False);
                if (surfaceAdaptor.GetType() != GeomAbs_Plane) {
                    continue;
                }
                if (LoopLiesOnFaceByUV(candidateFace, faceVertices, matchTolerance)) {
                    AddUniqueFaceCandidate(candidates, candidateFace);
                }
            } catch (const Standard_Failure&) {
                continue;
            }
        }
        
        // Remainder of the function is unchanged, copy it as is.
        if (candidates.empty()) {
            return TopoDS_Face();
        }
        if (candidates.size() == 1) {
            return candidates[0];
        }
        
        std::vector<TopoDS_Face> alignedCandidates;
        double maxDotAbs = -1.0;
        const double guideNormalSq = static_cast<double>(glm::length2(guideNormal));
        if (guideNormalSq > 1.0e-18) {
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

    
    bool Scene::ExtrudeFace(uint64_t objectId, const std::vector<glm::vec3>& faceVertices, const glm::vec3& direction, float distance, bool disableMerge) {
        SceneObject* obj = get_object_by_id(objectId);
        if (!obj || !obj->hasGeometry() || faceVertices.empty()) {
            return false;
        }

        // --- Mesh Mode: Direct mesh extrusion ---
        if (auto* meshGeom = dynamic_cast<MeshGeometry*>(obj->getGeometry())) {
            if (std::abs(distance) < 1e-4f) return true;

            // Find vertex indices from positions
            const auto& mesh = meshGeom->getMesh();
            std::set<unsigned int> faceIndices;
            size_t numVertices = mesh.vertices.size() / 3;

            for (const auto& faceVert : faceVertices) {
                for (size_t i = 0; i < numVertices; ++i) {
                    glm::vec3 meshVert(mesh.vertices[i * 3],
                                       mesh.vertices[i * 3 + 1],
                                       mesh.vertices[i * 3 + 2]);
                    if (glm::distance(faceVert, meshVert) < SCENE_POINT_EQUALITY_TOLERANCE) {
                        faceIndices.insert(static_cast<unsigned int>(i));
                        break;
                    }
                }
            }

            if (faceIndices.empty()) {
                std::cerr << "ExtrudeFace (Mesh): Could not find matching vertices." << std::endl;
                return false;
            }

            // Calculate actual face normal from mesh triangles
            glm::vec3 faceNormal(0.0f);
            size_t numTriangles = mesh.indices.size() / 3;
            int faceTriCount = 0;

            for (size_t t = 0; t < numTriangles; ++t) {
                unsigned int i0 = mesh.indices[t * 3 + 0];
                unsigned int i1 = mesh.indices[t * 3 + 1];
                unsigned int i2 = mesh.indices[t * 3 + 2];

                // Check if all vertices of this triangle are in the face
                if (faceIndices.count(i0) && faceIndices.count(i1) && faceIndices.count(i2)) {
                    glm::vec3 v0(mesh.vertices[i0 * 3], mesh.vertices[i0 * 3 + 1], mesh.vertices[i0 * 3 + 2]);
                    glm::vec3 v1(mesh.vertices[i1 * 3], mesh.vertices[i1 * 3 + 1], mesh.vertices[i1 * 3 + 2]);
                    glm::vec3 v2(mesh.vertices[i2 * 3], mesh.vertices[i2 * 3 + 1], mesh.vertices[i2 * 3 + 2]);

                    glm::vec3 triNormal = glm::cross(v1 - v0, v2 - v0);
                    faceNormal += triNormal; // Accumulate (weighted by area)
                    faceTriCount++;
                }
            }

            if (faceTriCount == 0 || glm::length(faceNormal) < 1e-6f) {
                std::cerr << "ExtrudeFace (Mesh): Could not calculate face normal." << std::endl;
                return false;
            }

            faceNormal = glm::normalize(faceNormal);

            // Determine extrusion direction based on user's direction hint
            // If user direction aligns with face normal, extrude along normal
            // If opposite, extrude in reverse
            float dotProduct = glm::dot(faceNormal, glm::normalize(direction));
            if (dotProduct < 0) {
                faceNormal = -faceNormal;
            }

            bool success = meshGeom->extrudeFace(faceIndices, faceNormal, distance);
            if (success) {
                meshGeom->recalculateNormals();
                obj->invalidateMeshCache();
                MarkStaticGeometryDirty();
            }
            return success;
        }

        // --- BRep Mode: Original OCCT logic ---
        auto* brepGeom = dynamic_cast<BRepGeometry*>(obj->getGeometry());
        if (!brepGeom) {
            std::cerr << "ExtrudeFace Error: Unsupported geometry type." << std::endl;
            return false;
        }
        const TopoDS_Shape* originalShape = brepGeom->getShape();
        if (!originalShape) return false;
        
        if (std::abs(distance) < 1e-4) {
            return true;
        }

        const auto preservedLineGraph = CaptureObjectPlanarLineGraph(obj);
        
        TopoDS_Face faceToExtrude = FindOriginalFace(obj, faceVertices, direction);
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
            if (!prismMaker.IsDone()) { std::cerr << "OCCT Error: Failed to create prism from face." << std::endl; return false; }
            
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

        ShapeFix_Shape shapeFixer(finalShape);
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
        
        // NEW: Update geometry through the new system
        brepGeom->setShape(CadKernel::OCCT_ShapeUniquePtr(new TopoDS_Shape(healedShape)));
        obj->invalidateMeshCache();
        UpdateObjectBoundary(obj);
        RestoreObjectPlanarLineGraph(obj, preservedLineGraph);
        
        } catch (const Standard_Failure& e) {
            std::cerr << "OCCT Exception during ExtrudeFace: " << e.GetMessageString() << std::endl;
            return false;
        }
        
        MarkStaticGeometryDirty();
        return true;
    }

    // --- NEW: Testing Infrastructure ---
    
    void Scene::ClearScene() { // This is now private
        // --- MODIFICATION START: Erase only exportable objects ---
        for (auto it = objects_.begin(); it != objects_.end(); ) {
            if (it->second->isExportable()) {
                // Also remove any boundary lines associated with the object being deleted
                for (uint64_t lineId : it->second->boundaryLineIDs) {
                    RemoveLine(lineId);
                }
                it = objects_.erase(it); // Erase and move to the next valid iterator
            } else {
                ++it; // Keep this object and move to the next one
            }
        }
        // --- MODIFICATION END ---
        
        // Reset counters and other scene data
        next_object_id_ = 1; // Resetting this might cause ID conflicts if we are not careful, but for "New Scene" it's ok.
        next_face_id_ = 1;
        
        lines_.clear();
        vertexAdjacency_.clear();
        next_line_id_ = 1;
        
        commandManager_->ClearHistory();
        if (materialManager_) { materialManager_->ClearMaterials(); }

        MarkStaticGeometryDirty();
        std::cout << "Scene: Cleared all user-created objects and lines." << std::endl;
    }

    void Scene::NewScene() {
        ClearScene();
    }

    bool Scene::SaveToStream(std::ostream& file) {
        // 2. Scene Objects
        uint64_t numObjects = objects_.size();
        file.write(reinterpret_cast<const char*>(&numObjects), sizeof(numObjects));

        for (const auto& [id, obj_ptr] : objects_) {
            // ID
            file.write(reinterpret_cast<const char*>(&id), sizeof(id));
            // Name
            std::string name = obj_ptr->get_name();
            uint32_t nameLen = static_cast<uint32_t>(name.length());
            file.write(reinterpret_cast<const char*>(&nameLen), sizeof(nameLen));
            file.write(name.c_str(), nameLen);
            
            // Shape
            bool hasShape = false;
            if (auto* brepGeom = dynamic_cast<BRepGeometry*>(obj_ptr->getGeometry())) {
                const TopoDS_Shape* shape = brepGeom->getShape();
                hasShape = (shape && !shape->IsNull());
                file.write(reinterpret_cast<const char*>(&hasShape), sizeof(hasShape));
                if (hasShape) {
                    try {
                        BinTools::Write(*shape, file);
                    } catch(...) {
                        std::cerr << "Scene Error: Failed to write shape for object ID " << id << std::endl;
                        // We can't easily recover here, so maybe it's best to fail the save.
                        return false;
                    }
                }
            } else {
                file.write(reinterpret_cast<const char*>(&hasShape), sizeof(hasShape));
            }
        }

        std::cout << "Scene: Saved " << numObjects << " objects to stream." << std::endl;
        return true;
    }

    bool Scene::LoadFromStream(std::istream& file) {
        ClearScene();

        uint64_t numObjects;
        file.read(reinterpret_cast<char*>(&numObjects), sizeof(numObjects));
        if (!file) return false;

        for (uint64_t i = 0; i < numObjects; ++i) {
            uint64_t id;
            file.read(reinterpret_cast<char*>(&id), sizeof(id));

            uint32_t nameLen;
            file.read(reinterpret_cast<char*>(&nameLen), sizeof(nameLen));
            std::string name(nameLen, '\0');
            file.read(&name[0], nameLen);
            
            SceneObject* newObj = create_object_with_id(id, name);
            if (!newObj) continue;

            bool hasShape;
            file.read(reinterpret_cast<char*>(&hasShape), sizeof(hasShape));
            if (hasShape) {
                TopoDS_Shape shape;
                try {
                    BinTools::Read(shape, file);
                    // NEW: Use the new geometry system
                    auto geometry = std::make_unique<BRepGeometry>(CadKernel::OCCT_ShapeUniquePtr(new TopoDS_Shape(shape)));
                    newObj->setGeometry(std::move(geometry));
                    UpdateObjectBoundary(newObj);
                    // The mesh will be created lazily by getMeshBuffers()
                } catch (...) {
                     std::cerr << "Scene Error: Failed to read shape for object ID " << id << std::endl;
                }
            }
        }

        std::cout << "Scene: Loaded " << objects_.size() << " objects from stream." << std::endl;
        return true;
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
            if (!wireMaker.IsDone()) { std::cerr << "CreateRectangularFace Error: Failed to create wire." << std::endl; return nullptr; }
            
            TopoDS_Wire wire = wireMaker.Wire();
            BRepBuilderAPI_MakeFace faceMaker(wire, Standard_True);
            if (!faceMaker.IsDone() || faceMaker.Face().IsNull()) { std::cerr << "CreateRectangularFace Error: Failed to create face." << std::endl; return nullptr; }

            SceneObject* new_face_obj = this->create_object(name);
            if (new_face_obj) {
                // NEW: Use the new geometry system
                auto geometry = std::make_unique<BRepGeometry>(CadKernel::OCCT_ShapeUniquePtr(new TopoDS_Shape(faceMaker.Face())));
                new_face_obj->setGeometry(std::move(geometry));

                UpdateObjectBoundary(new_face_obj);
                std::cout << "Scene: Created test face object: " << name << std::endl;
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
            auto* brepGeom = dynamic_cast<BRepGeometry*>(finalObj->getGeometry());
            if (!brepGeom || !brepGeom->getShape()) {
                std::cout << "TEST FAILED: Final object has no B-Rep geometry." << std::endl;
                return;
            }
            TopExp_Explorer faceExp(*brepGeom->getShape(), TopAbs_FACE);
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
            if (auto* brepGeom = dynamic_cast<BRepGeometry*>(obj_ptr->getGeometry())) {
                const TopoDS_Shape* shape = brepGeom->getShape();
                if (shape) {
                    AnalyzeShape(*shape, "Final Object " + std::to_string(id));
                }
            }
        }
        
        std::cout << "\n--- FACE SPLITTING TEST FINISHED ---\n" << std::endl;
    }

    SceneObject* Scene::FindObjectByFace(const std::vector<glm::vec3>& faceVertices) {
        if (faceVertices.empty()) return nullptr;
        
        for (auto* obj : get_all_objects()) {
            if (!obj || !obj->hasGeometry()) continue;
            // Guide normal doesn't matter much here, we're just checking for containment.
            TopoDS_Face foundFace = FindOriginalFace(obj, faceVertices, glm::vec3(0,0,1));
            if (!foundFace.IsNull()) {
                return obj;
            }
        }
        return nullptr;
    }

    bool Scene::RebuildObjectByMovingVertices(
        uint64_t objectId,
        Engine::SubObjectType type,
        const std::vector<glm::vec3>& initialPositions,
        const glm::vec3& translation,
        const SubObjectMovePreviewMesh* previewMesh
    ) {
        SceneObject* obj = get_object_by_id(objectId);
        if (!obj || !obj->hasGeometry()) return false;

        // --- Mesh Mode: Direct vertex movement ---
        if (auto* meshGeom = dynamic_cast<MeshGeometry*>(obj->getGeometry())) {
            const auto& mesh = meshGeom->getMesh();
            std::set<unsigned int> indicesToMove;
            size_t numVertices = mesh.vertices.size() / 3;

            // Find vertex indices from positions
            for (const auto& pos : initialPositions) {
                for (size_t i = 0; i < numVertices; ++i) {
                    glm::vec3 meshVert(mesh.vertices[i * 3],
                                       mesh.vertices[i * 3 + 1],
                                       mesh.vertices[i * 3 + 2]);
                    if (glm::distance(pos, meshVert) < SCENE_POINT_EQUALITY_TOLERANCE) {
                        indicesToMove.insert(static_cast<unsigned int>(i));
                        break;
                    }
                }
            }

            if (indicesToMove.empty()) {
                return false;
            }

            meshGeom->moveVertices(indicesToMove, translation);
            meshGeom->recalculateNormals();
            obj->invalidateMeshCache();
            UpdateObjectBoundary(obj);
            MarkStaticGeometryDirty();
            return true;
        }

        // --- BRep Mode: topology-aware SketchUp-style local move ---
        auto* brepGeom = dynamic_cast<BRepGeometry*>(obj->getGeometry());
        if (!brepGeom) {
            std::cout << "Warning: Vertex move reconstruction is only supported for B-Rep or Mesh objects." << std::endl;
            return false;
        }
        const TopoDS_Shape* originalShape = brepGeom->getShape();
        if (!originalShape || originalShape->IsNull()) {
            std::cerr << "Rebuild Error: Object has no B-Rep shape." << std::endl;
            return false;
        }

        if (glm::length2(translation) < 1.0e-12f) {
            return false;
        }

        const double matchTolerance = ShapeMatchTolerance(*originalShape);
        const auto preservedLineGraph = CaptureObjectPlanarLineGraph(obj);

        if (type != SubObjectType::FACE &&
            previewMesh &&
            !previewMesh->mesh.isEmpty() &&
            !previewMesh->movingVertexIndices.empty() &&
            ShouldPreferPreviewTopologyMove(*originalShape, previewMesh->mesh)) {
            bool previewChanged = false;
            TopoDS_Shape previewMovedShape = BuildMovedShapeFromPreviewMesh(
                previewMesh->mesh,
                previewMesh->movingVertexIndices,
                gp_Vec(translation.x, translation.y, translation.z),
                previewChanged);
            if (previewChanged && !previewMovedShape.IsNull()) {
                TopoDS_Shape finalPreviewShape = HealMovedShape(previewMovedShape, *originalShape, matchTolerance);
                if (!finalPreviewShape.IsNull() && BRepCheck_Analyzer(finalPreviewShape).IsValid()) {
                    std::cout << "MoveSubObject: finalized from preview mesh topology. vertices="
                              << (previewMesh->mesh.vertices.size() / 3)
                              << ", triangles=" << (previewMesh->mesh.indices.size() / 3)
                              << ", movedVertices=" << previewMesh->movingVertexIndices.size()
                              << std::endl;
                    brepGeom->setShape(CadKernel::OCCT_ShapeUniquePtr(new TopoDS_Shape(finalPreviewShape)));
                    obj->invalidateMeshCache();
                    UpdateObjectBoundary(obj);
                    RestoreObjectPlanarLineGraph(obj, preservedLineGraph);
                    MarkStaticGeometryDirty();
                    return true;
                }

                std::cout << "MoveSubObject Warning: preview mesh topology rebuild was invalid; falling back to B-Rep topology rebuild." << std::endl;
            }
        }

        TopTools_IndexedMapOfShape selectedVertices;
        std::vector<glm::vec3> selectedTopologyPositions;

        if (type == SubObjectType::FACE) {
            TopoDS_Face faceToMove = FindOriginalFace(obj, initialPositions, translation);
            if (faceToMove.IsNull()) {
                std::cout << "MoveSubObject Error: Could not locate B-Rep face to move." << std::endl;
                return false;
            }

            TopExp::MapShapes(faceToMove, TopAbs_VERTEX, selectedVertices);
            selectedTopologyPositions = VertexPositionsOfShape(faceToMove);
            for (const glm::vec3& inputPosition : initialPositions) {
                AddUniquePosition(selectedTopologyPositions, inputPosition, SCENE_POINT_EQUALITY_TOLERANCE);
            }

            TopTools_IndexedMapOfShape verticesNearFace = FindVerticesNearPositions(*originalShape, selectedTopologyPositions, matchTolerance);
            AddVerticesFromMap(selectedVertices, verticesNearFace);
        } else {
            selectedTopologyPositions = initialPositions;
            selectedVertices = FindVerticesNearPositions(*originalShape, initialPositions, matchTolerance);
        }

        std::cout << "MoveSubObject: type=" << SubObjectTypeName(type)
                  << ", inputPositions=" << initialPositions.size()
                  << ", topologyPositions=" << selectedTopologyPositions.size()
                  << ", resolvedBRepVertices=" << selectedVertices.Extent()
                  << ", tolerance=" << matchTolerance
                  << std::endl;

        if (selectedVertices.Extent() == 0 && selectedTopologyPositions.empty()) {
            std::cout << "MoveSubObject Error: Could not resolve selected render vertices to B-Rep vertices." << std::endl;
            return false;
        }

        gp_Vec occtTranslation(translation.x, translation.y, translation.z);
        int matchedFaceCount = 0;
        int changedFaceCount = 0;

        try {
            TopoDS_Shape movedShape = BuildMovedShapeFromFaces(*originalShape, selectedVertices, selectedTopologyPositions, occtTranslation, matchTolerance, matchedFaceCount, changedFaceCount);
            if (changedFaceCount == 0) {
                std::cout << "MoveSubObject Error: Selection did not affect any B-Rep faces. type="
                          << SubObjectTypeName(type)
                          << ", inputPositions=" << initialPositions.size()
                          << ", topologyPositions=" << selectedTopologyPositions.size()
                          << ", resolvedBRepVertices=" << selectedVertices.Extent()
                          << ", matchedFaces=" << matchedFaceCount
                          << std::endl;
                return false;
            }

            if (movedShape.IsNull()) {
                std::cout << "MoveSubObject Error: Could not rebuild moved B-Rep face compound." << std::endl;
                return false;
            }

            TopoDS_Shape finalShape = HealMovedShape(movedShape, *originalShape, matchTolerance);
            if (finalShape.IsNull() || !BRepCheck_Analyzer(finalShape).IsValid()) {
                std::cout << "MoveSubObject Error: Final B-Rep shape is invalid after local move." << std::endl;
                return false;
            }

            brepGeom->setShape(CadKernel::OCCT_ShapeUniquePtr(new TopoDS_Shape(finalShape)));
            obj->invalidateMeshCache();
            UpdateObjectBoundary(obj);
            RestoreObjectPlanarLineGraph(obj, preservedLineGraph);
            MarkStaticGeometryDirty();
            return true;
        } catch (const Standard_Failure& e) {
            std::cerr << "MoveSubObject Error: OCCT exception during local move: " << e.GetMessageString() << std::endl;
        }
        return false;
    }

    // --- NEW: Implementation of dirty flag methods ---
    void Scene::MarkStaticGeometryDirty() {
        isStaticGeometryDirty_ = true;
    }

    bool Scene::IsStaticGeometryDirty() const {
        return isStaticGeometryDirty_;
    }

    void Scene::ClearStaticGeometryDirtyFlag() {
        isStaticGeometryDirty_ = false;
    }

} // namespace Urbaxio
