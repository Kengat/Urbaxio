#include "engine/geometry/BRepGeometry.h"
#include "engine/scene.h"
#include "engine/scene_object.h"

#include <BRepCheck_Analyzer.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <TopAbs.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>

#include <glm/glm.hpp>

#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

int CountFaces(const Urbaxio::Engine::SceneObject* object)
{
    if (!object) {
        return 0;
    }
    auto* brep = dynamic_cast<Urbaxio::Engine::BRepGeometry*>(object->getGeometry());
    if (!brep || !brep->getShape() || brep->getShape()->IsNull()) {
        return 0;
    }

    int faceCount = 0;
    for (TopExp_Explorer ex(*brep->getShape(), TopAbs_FACE); ex.More(); ex.Next()) {
        ++faceCount;
    }
    return faceCount;
}

double TopPlanarArea(const Urbaxio::Engine::SceneObject* object, float z)
{
    auto* brep = object ? dynamic_cast<Urbaxio::Engine::BRepGeometry*>(object->getGeometry()) : nullptr;
    if (!brep || !brep->getShape() || brep->getShape()->IsNull()) {
        return 0.0;
    }

    double area = 0.0;
    for (TopExp_Explorer ex(*brep->getShape(), TopAbs_FACE); ex.More(); ex.Next()) {
        const TopoDS_Face face = TopoDS::Face(ex.Current());
        GProp_GProps props;
        BRepGProp::SurfaceProperties(face, props);
        const gp_Pnt center = props.CentreOfMass();
        if (std::abs(center.Z() - z) <= 1.0e-4) {
            area += props.Mass();
        }
    }
    return area;
}

double TopRenderSignedArea(const Urbaxio::Engine::SceneObject* object, float z)
{
    if (!object) {
        return 0.0;
    }

    const auto& mesh = object->getMeshBuffers();
    double signedArea = 0.0;
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        const unsigned int ia = mesh.indices[i + 0];
        const unsigned int ib = mesh.indices[i + 1];
        const unsigned int ic = mesh.indices[i + 2];
        if (static_cast<size_t>(ia) * 3 + 2 >= mesh.vertices.size() ||
            static_cast<size_t>(ib) * 3 + 2 >= mesh.vertices.size() ||
            static_cast<size_t>(ic) * 3 + 2 >= mesh.vertices.size()) {
            continue;
        }

        const glm::vec3 a(mesh.vertices[ia * 3 + 0], mesh.vertices[ia * 3 + 1], mesh.vertices[ia * 3 + 2]);
        const glm::vec3 b(mesh.vertices[ib * 3 + 0], mesh.vertices[ib * 3 + 1], mesh.vertices[ib * 3 + 2]);
        const glm::vec3 c(mesh.vertices[ic * 3 + 0], mesh.vertices[ic * 3 + 1], mesh.vertices[ic * 3 + 2]);
        if (std::abs(a.z - z) > 1.0e-4f ||
            std::abs(b.z - z) > 1.0e-4f ||
            std::abs(c.z - z) > 1.0e-4f) {
            continue;
        }

        signedArea += 0.5 * static_cast<double>((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
    }
    return signedArea;
}

void DumpTopRenderTriangles(const Urbaxio::Engine::SceneObject* object, float z)
{
    if (!object) {
        return;
    }

    const auto& mesh = object->getMeshBuffers();
    double signedArea = 0.0;
    double absoluteArea = 0.0;
    int topTriangles = 0;
    int reversedTriangles = 0;
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        const unsigned int ia = mesh.indices[i + 0];
        const unsigned int ib = mesh.indices[i + 1];
        const unsigned int ic = mesh.indices[i + 2];
        if (static_cast<size_t>(ia) * 3 + 2 >= mesh.vertices.size() ||
            static_cast<size_t>(ib) * 3 + 2 >= mesh.vertices.size() ||
            static_cast<size_t>(ic) * 3 + 2 >= mesh.vertices.size()) {
            continue;
        }

        const glm::vec3 a(mesh.vertices[ia * 3 + 0], mesh.vertices[ia * 3 + 1], mesh.vertices[ia * 3 + 2]);
        const glm::vec3 b(mesh.vertices[ib * 3 + 0], mesh.vertices[ib * 3 + 1], mesh.vertices[ib * 3 + 2]);
        const glm::vec3 c(mesh.vertices[ic * 3 + 0], mesh.vertices[ic * 3 + 1], mesh.vertices[ic * 3 + 2]);
        if (std::abs(a.z - z) > 1.0e-4f ||
            std::abs(b.z - z) > 1.0e-4f ||
            std::abs(c.z - z) > 1.0e-4f) {
            continue;
        }

        const double area = 0.5 * static_cast<double>((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
        signedArea += area;
        absoluteArea += std::abs(area);
        ++topTriangles;
        if (area < 0.0) {
            ++reversedTriangles;
        }
        std::cerr << "  top tri " << (i / 3) << " area=" << area
                  << " a=(" << a.x << "," << a.y << "," << a.z << ")"
                  << " b=(" << b.x << "," << b.y << "," << b.z << ")"
                  << " c=(" << c.x << "," << c.y << "," << c.z << ")\n";
    }

    std::cerr << "  top tri stats: count=" << topTriangles
              << " reversed=" << reversedTriangles
              << " signed=" << signedArea
              << " abs=" << absoluteArea << "\n";
}

bool ValidateBRep(const Urbaxio::Engine::SceneObject* object, const char* label)
{
    auto* brep = object ? dynamic_cast<Urbaxio::Engine::BRepGeometry*>(object->getGeometry()) : nullptr;
    if (!brep || !brep->getShape() || brep->getShape()->IsNull()) {
        std::cerr << label << ": missing B-Rep shape\n";
        return false;
    }

    BRepCheck_Analyzer analyzer(*brep->getShape());
    if (!analyzer.IsValid()) {
        std::cerr << label << ": invalid B-Rep after face split\n";
        return false;
    }
    return true;
}

bool DrawLoop(Urbaxio::Engine::Scene& scene, const std::vector<glm::vec3>& points)
{
    if (points.size() < 3) {
        return false;
    }

    for (size_t i = 0; i < points.size(); ++i) {
        scene.AddUserLine(points[i], points[(i + 1) % points.size()]);
    }
    return true;
}

void AddUniquePoint(std::vector<glm::vec3>& points, const glm::vec3& point)
{
    for (const glm::vec3& existing : points) {
        if (glm::distance(existing, point) <= Urbaxio::SCENE_POINT_EQUALITY_TOLERANCE) {
            return;
        }
    }
    points.push_back(point);
}

std::vector<glm::vec3> CollectFaceVerticesNear(
    const Urbaxio::Engine::SceneObject& object,
    const glm::vec3& target)
{
    const auto& mesh = object.getMeshBuffers();
    size_t bestTriangle = static_cast<size_t>(-1);
    float bestDistanceSq = std::numeric_limits<float>::max();

    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        glm::vec3 center(0.0f);
        bool valid = true;
        for (int corner = 0; corner < 3; ++corner) {
            const unsigned int index = mesh.indices[i + corner];
            if (static_cast<size_t>(index) * 3 + 2 >= mesh.vertices.size()) {
                valid = false;
                break;
            }
            center += glm::vec3(
                mesh.vertices[index * 3 + 0],
                mesh.vertices[index * 3 + 1],
                mesh.vertices[index * 3 + 2]);
        }
        if (!valid) {
            continue;
        }
        center /= 3.0f;

        const glm::vec3 delta = center - target;
        const float distanceSq = glm::dot(delta, delta);
        if (distanceSq < bestDistanceSq) {
            bestDistanceSq = distanceSq;
            bestTriangle = i;
        }
    }

    std::vector<glm::vec3> vertices;
    if (bestTriangle == static_cast<size_t>(-1)) {
        return vertices;
    }

    const int faceId = object.getFaceIdForTriangle(bestTriangle);
    const std::vector<size_t>* faceTriangles = object.getFaceTriangles(faceId);
    const std::vector<size_t> fallback = {bestTriangle};
    const std::vector<size_t>& triangles = faceTriangles ? *faceTriangles : fallback;

    for (size_t triBase : triangles) {
        if (triBase + 2 >= mesh.indices.size()) {
            continue;
        }
        for (int corner = 0; corner < 3; ++corner) {
            const unsigned int index = mesh.indices[triBase + corner];
            if (static_cast<size_t>(index) * 3 + 2 >= mesh.vertices.size()) {
                continue;
            }
            AddUniquePoint(vertices, {
                mesh.vertices[index * 3 + 0],
                mesh.vertices[index * 3 + 1],
                mesh.vertices[index * 3 + 2],
            });
        }
    }

    return vertices;
}

void DrawPolyline(Urbaxio::Engine::Scene& scene, const std::vector<glm::vec3>& points)
{
    for (size_t i = 1; i < points.size(); ++i) {
        scene.AddUserLine(points[i - 1], points[i]);
    }
}

size_t CountUserDrawnLines(const Urbaxio::Engine::Scene& scene)
{
    size_t count = 0;
    for (const auto& [id, line] : scene.GetAllLines()) {
        if (line.isUserDrawn) {
            ++count;
        }
    }
    return count;
}

bool HasOrphanGeneratedLine(const Urbaxio::Engine::Scene& scene, const Urbaxio::Engine::SceneObject& object)
{
    for (const auto& [id, line] : scene.GetAllLines()) {
        if (!line.isUserDrawn && object.boundaryLineIDs.count(id) == 0) {
            return true;
        }
    }
    return false;
}

bool RunInteriorSquareSplitCase(const char* label, float zOffset)
{
    Urbaxio::Engine::Scene scene;
    Urbaxio::Engine::SceneObject* box = scene.create_box_object(std::string(label) + "_box", 2.0, 2.0, 2.0);
    if (!box || !ValidateBRep(box, label)) {
        return false;
    }

    const uint64_t boxId = box->get_id();
    const int beforeFaceCount = CountFaces(box);
    const size_t beforeObjectCount = scene.get_all_objects().size();
    const float z = 1.0f + zOffset;

    DrawLoop(scene, {
        {-0.45f, -0.45f, z},
        { 0.45f, -0.45f, z},
        { 0.45f,  0.45f, z},
        {-0.45f,  0.45f, z},
    });

    Urbaxio::Engine::SceneObject* splitBox = scene.get_object_by_id(boxId);
    const size_t afterObjectCount = scene.get_all_objects().size();
    const int afterFaceCount = CountFaces(splitBox);

    if (afterObjectCount != beforeObjectCount) {
        std::cerr << label << ": expected split to stay on host object, object count "
                  << beforeObjectCount << " -> " << afterObjectCount << "\n";
        return false;
    }
    if (afterFaceCount <= beforeFaceCount) {
        std::cerr << label << ": expected more faces after split, face count "
                  << beforeFaceCount << " -> " << afterFaceCount << "\n";
        return false;
    }
    if (!ValidateBRep(splitBox, label)) {
        return false;
    }

    std::cout << label << " passed: faces " << beforeFaceCount << " -> " << afterFaceCount << "\n";
    return true;
}

bool RunMoveNewSplitFaceCase()
{
    Urbaxio::Engine::Scene scene;
    Urbaxio::Engine::SceneObject* box = scene.create_box_object("split_then_move_box", 2.0, 2.0, 2.0);
    if (!box || !ValidateBRep(box, "split_then_move_setup")) {
        return false;
    }

    const uint64_t boxId = box->get_id();
    DrawLoop(scene, {
        {-0.45f, -0.45f, 1.0f},
        { 0.45f, -0.45f, 1.0f},
        { 0.45f,  0.45f, 1.0f},
        {-0.45f,  0.45f, 1.0f},
    });

    box = scene.get_object_by_id(boxId);
    if (!box || CountFaces(box) < 7 || !ValidateBRep(box, "split_then_move_after_split")) {
        std::cerr << "split_then_move: split setup failed\n";
        return false;
    }

    const auto& mesh = box->getMeshBuffers();
    size_t bestTriangle = static_cast<size_t>(-1);
    float bestDistanceSq = std::numeric_limits<float>::max();

    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        glm::vec3 center(0.0f);
        bool valid = true;
        for (int corner = 0; corner < 3; ++corner) {
            const unsigned int index = mesh.indices[i + corner];
            if (static_cast<size_t>(index) * 3 + 2 >= mesh.vertices.size()) {
                valid = false;
                break;
            }
            center += glm::vec3(
                mesh.vertices[index * 3 + 0],
                mesh.vertices[index * 3 + 1],
                mesh.vertices[index * 3 + 2]);
        }
        if (!valid) {
            continue;
        }
        center /= 3.0f;

        if (std::abs(center.z - 1.0f) > 1.0e-3f ||
            std::abs(center.x) > 0.45f ||
            std::abs(center.y) > 0.45f) {
            continue;
        }

        const float distanceSq = glm::dot(center, center);
        if (distanceSq < bestDistanceSq) {
            bestDistanceSq = distanceSq;
            bestTriangle = i;
        }
    }

    if (bestTriangle == static_cast<size_t>(-1)) {
        std::cerr << "split_then_move: could not find render triangle on the new split face\n";
        return false;
    }

    std::vector<glm::vec3> selectedPositions;
    for (int corner = 0; corner < 3; ++corner) {
        const unsigned int index = mesh.indices[bestTriangle + corner];
        selectedPositions.emplace_back(
            mesh.vertices[index * 3 + 0],
            mesh.vertices[index * 3 + 1],
            mesh.vertices[index * 3 + 2]);
    }

    const int beforeMoveFaces = CountFaces(box);
    if (!scene.RebuildObjectByMovingVertices(
            boxId,
            Urbaxio::Engine::SubObjectType::FACE,
            selectedPositions,
            glm::vec3(0.16f, 0.08f, 0.0f))) {
        std::cerr << "split_then_move: moving the newly split face returned false\n";
        return false;
    }

    box = scene.get_object_by_id(boxId);
    if (!box || CountFaces(box) < beforeMoveFaces || !ValidateBRep(box, "split_then_move_after_move")) {
        return false;
    }

    std::cout << "split_then_move_new_face passed: faces " << beforeMoveFaces
              << " -> " << CountFaces(box) << "\n";
    return true;
}

bool RunOpenSegmentSplitCase()
{
    Urbaxio::Engine::Scene scene;
    Urbaxio::Engine::SceneObject* box = scene.create_box_object("open_segment_split_box", 2.0, 2.0, 2.0);
    if (!box || !ValidateBRep(box, "open_segment_split_setup")) {
        return false;
    }

    const uint64_t boxId = box->get_id();
    const int beforeFaceCount = CountFaces(box);
    const size_t beforeObjectCount = scene.get_all_objects().size();

    scene.AddUserLine({-1.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 1.0f});

    box = scene.get_object_by_id(boxId);
    const int afterFaceCount = CountFaces(box);
    if (scene.get_all_objects().size() != beforeObjectCount) {
        std::cerr << "open_segment_split: expected split to stay on host object\n";
        return false;
    }
    if (afterFaceCount <= beforeFaceCount) {
        std::cerr << "open_segment_split: expected boundary-to-boundary line to split a face, face count "
                  << beforeFaceCount << " -> " << afterFaceCount << "\n";
        return false;
    }
    if (!ValidateBRep(box, "open_segment_split_after")) {
        return false;
    }

    std::cout << "open_segment_split passed: faces " << beforeFaceCount
              << " -> " << afterFaceCount << "\n";
    return true;
}

bool RunOverlappingSegmentNodingCase()
{
    Urbaxio::Engine::Scene scene;
    Urbaxio::Engine::SceneObject* box = scene.create_box_object("overlap_noding_box", 2.0, 2.0, 2.0);
    if (!box || !ValidateBRep(box, "overlap_noding_setup")) {
        return false;
    }

    const uint64_t boxId = box->get_id();
    const int beforeFaceCount = CountFaces(box);
    const size_t beforeObjectCount = scene.get_all_objects().size();

    scene.AddUserLine({-1.0f, 0.0f, 1.0f}, { 1.0f, 0.0f, 1.0f});
    scene.AddUserLine({-0.5f, 0.0f, 1.0f}, { 0.5f, 0.0f, 1.0f});
    scene.AddUserLine({ 0.0f,-1.0f, 1.0f}, { 0.0f, 1.0f, 1.0f});

    box = scene.get_object_by_id(boxId);
    const int afterFaceCount = CountFaces(box);
    if (scene.get_all_objects().size() != beforeObjectCount) {
        std::cerr << "overlap_noding: expected all splits to stay on host object\n";
        return false;
    }
    if (afterFaceCount < beforeFaceCount + 3) {
        std::cerr << "overlap_noding: expected overlapped graph to remain noded and splittable, face count "
                  << beforeFaceCount << " -> " << afterFaceCount << "\n";
        return false;
    }
    if (!ValidateBRep(box, "overlap_noding_after")) {
        return false;
    }

    std::cout << "overlap_noding passed: faces " << beforeFaceCount
              << " -> " << afterFaceCount << "\n";
    return true;
}

bool RunEnvelopeGraphSplitCase()
{
    Urbaxio::Engine::Scene scene;
    Urbaxio::Engine::SceneObject* box = scene.create_box_object("envelope_graph_box", 2.0, 2.0, 2.0);
    if (!box || !ValidateBRep(box, "envelope_graph_setup")) {
        return false;
    }

    const uint64_t boxId = box->get_id();
    const int beforeFaceCount = CountFaces(box);
    const size_t beforeObjectCount = scene.get_all_objects().size();
    const float z = 1.0f;

    scene.AddUserLine({-1.0f, -1.0f, z}, { 1.0f,  1.0f, z});
    scene.AddUserLine({-1.0f,  1.0f, z}, { 1.0f, -1.0f, z});

    DrawLoop(scene, {
        {-0.36f,  0.36f, z},
        { 0.00f,  0.66f, z},
        { 0.36f,  0.36f, z},
        { 0.00f, -0.08f, z},
    });

    DrawPolyline(scene, {
        {-1.0f, -1.0f, z},
        { 0.00f, -0.08f, z},
        { 1.0f, -1.0f, z},
    });

    box = scene.get_object_by_id(boxId);
    const size_t afterObjectCount = scene.get_all_objects().size();
    const int afterFaceCount = CountFaces(box);

    if (afterObjectCount != beforeObjectCount) {
        std::cerr << "envelope_graph: expected graph split to stay on host object, object count "
                  << beforeObjectCount << " -> " << afterObjectCount << "\n";
        return false;
    }
    if (afterFaceCount < beforeFaceCount + 5) {
        std::cerr << "envelope_graph: expected a rich planar partition, face count "
                  << beforeFaceCount << " -> " << afterFaceCount << "\n";
        return false;
    }
    if (!ValidateBRep(box, "envelope_graph_after_split")) {
        return false;
    }

    std::cout << "envelope_graph_split passed: faces " << beforeFaceCount
              << " -> " << afterFaceCount << "\n";
    return true;
}

bool RunPartialEnvelopeKeepsTopFaceCase()
{
    Urbaxio::Engine::Scene scene;
    Urbaxio::Engine::SceneObject* box = scene.create_box_object("partial_envelope_box", 2.0, 2.0, 2.0);
    if (!box || !ValidateBRep(box, "partial_envelope_setup")) {
        return false;
    }

    const uint64_t boxId = box->get_id();
    const float z = 1.0f;
    const double initialTopArea = TopPlanarArea(box, z);

    scene.AddUserLine({-1.0f, -1.0f, z}, { 1.0f,  1.0f, z});
    scene.AddUserLine({-1.0f,  1.0f, z}, { 1.0f, -1.0f, z});
    scene.AddUserLine({-1.0f, -1.0f, z}, { 0.0f,  0.42f, z});

    box = scene.get_object_by_id(boxId);
    const double afterTopArea = TopPlanarArea(box, z);
    const double afterRenderSignedArea = TopRenderSignedArea(box, z);
    if (!box || !ValidateBRep(box, "partial_envelope_after_third_line")) {
        return false;
    }
    if (std::abs(afterTopArea - initialTopArea) > 1.0e-3) {
        std::cerr << "partial_envelope: top face area changed after incomplete envelope, area "
                  << initialTopArea << " -> " << afterTopArea << "\n";
        return false;
    }
    if (std::abs(std::abs(afterRenderSignedArea) - initialTopArea) > 1.0e-3) {
        std::cerr << "partial_envelope: top render triangles have inconsistent winding after third line, signed area "
                  << afterRenderSignedArea << " expected magnitude " << initialTopArea << "\n";
        DumpTopRenderTriangles(box, z);
        return false;
    }

    std::cout << "partial_envelope_keeps_top_face passed: top area "
              << initialTopArea << " -> " << afterTopArea
              << ", signedRenderArea=" << afterRenderSignedArea
              << ", faces=" << CountFaces(box) << "\n";
    return true;
}

bool RunEnvelopePushPullPreservesGraphCase()
{
    Urbaxio::Engine::Scene scene;
    Urbaxio::Engine::SceneObject* box = scene.create_box_object("envelope_pushpull_box", 2.0, 2.0, 2.0);
    if (!box || !ValidateBRep(box, "envelope_pushpull_setup")) {
        return false;
    }

    const uint64_t boxId = box->get_id();
    const float z = 1.0f;

    scene.AddUserLine({-1.0f, -1.0f, z}, { 1.0f,  1.0f, z});
    scene.AddUserLine({-1.0f,  1.0f, z}, { 1.0f, -1.0f, z});

    const std::vector<glm::vec3> diamond = {
        {-0.36f,  0.36f, z},
        { 0.00f,  0.66f, z},
        { 0.36f,  0.36f, z},
        { 0.00f, -0.08f, z},
    };
    DrawLoop(scene, diamond);

    DrawPolyline(scene, {
        {-1.0f, -1.0f, z},
        { 0.00f, -0.08f, z},
        { 1.0f, -1.0f, z},
    });

    box = scene.get_object_by_id(boxId);
    const int beforeExtrudeFaces = CountFaces(box);
    const size_t beforeExtrudeUserLines = CountUserDrawnLines(scene);
    if (beforeExtrudeFaces < 12 || !ValidateBRep(box, "envelope_pushpull_before_extrude")) {
        std::cerr << "envelope_pushpull: envelope graph setup did not create enough faces\n";
        return false;
    }

    const std::vector<glm::vec3> faceToExtrude = CollectFaceVerticesNear(*box, glm::vec3(0.0f, 0.42f, z));
    if (faceToExtrude.size() < 3) {
        std::cerr << "envelope_pushpull: could not collect central graph face vertices\n";
        return false;
    }

    if (!scene.ExtrudeFace(boxId, faceToExtrude, glm::vec3(0.0f, 0.0f, 1.0f), 0.35f, false)) {
        std::cerr << "envelope_pushpull: ExtrudeFace returned false\n";
        return false;
    }

    box = scene.get_object_by_id(boxId);
    const int afterExtrudeFaces = CountFaces(box);
    const size_t afterExtrudeUserLines = CountUserDrawnLines(scene);
    if (afterExtrudeFaces < beforeExtrudeFaces) {
        std::cerr << "envelope_pushpull: expected existing graph faces to survive extrusion, face count "
                  << beforeExtrudeFaces << " -> " << afterExtrudeFaces << "\n";
        return false;
    }
    if (afterExtrudeUserLines < beforeExtrudeUserLines) {
        std::cerr << "envelope_pushpull: user line graph lost segments after central extrusion, lines "
                  << beforeExtrudeUserLines << " -> " << afterExtrudeUserLines << "\n";
        return false;
    }
    if (!ValidateBRep(box, "envelope_pushpull_after_extrude")) {
        return false;
    }

    std::cout << "envelope_pushpull_preserves_graph passed: faces "
              << beforeExtrudeFaces << " -> " << afterExtrudeFaces << "\n";
    return true;
}

bool RunSidePushPullDropsOldBoundaryLinesCase()
{
    Urbaxio::Engine::Scene scene;
    Urbaxio::Engine::SceneObject* box = scene.create_box_object("side_pushpull_box", 2.0, 2.0, 2.0);
    if (!box || !ValidateBRep(box, "side_pushpull_setup")) {
        return false;
    }

    const uint64_t boxId = box->get_id();
    const float z = 1.0f;

    scene.AddUserLine({-1.0f, -1.0f, z}, { 1.0f,  1.0f, z});
    scene.AddUserLine({-1.0f,  1.0f, z}, { 1.0f, -1.0f, z});
    DrawLoop(scene, {
        {-0.36f,  0.36f, z},
        { 0.00f,  0.66f, z},
        { 0.36f,  0.36f, z},
        { 0.00f, -0.08f, z},
    });
    DrawPolyline(scene, {
        {-1.0f, -1.0f, z},
        { 0.00f, -0.08f, z},
        { 1.0f, -1.0f, z},
    });

    box = scene.get_object_by_id(boxId);
    if (!box || CountFaces(box) < 12 || !ValidateBRep(box, "side_pushpull_before_extrude")) {
        std::cerr << "side_pushpull: envelope graph setup failed\n";
        return false;
    }

    const std::vector<glm::vec3> faceToExtrude = CollectFaceVerticesNear(*box, glm::vec3(0.64f, 0.22f, z));
    if (faceToExtrude.size() < 3) {
        std::cerr << "side_pushpull: could not collect side graph face vertices\n";
        return false;
    }

    if (!scene.ExtrudeFace(boxId, faceToExtrude, glm::vec3(0.0f, 0.0f, 1.0f), 0.35f, false)) {
        std::cerr << "side_pushpull: ExtrudeFace returned false\n";
        return false;
    }

    box = scene.get_object_by_id(boxId);
    if (!box || !ValidateBRep(box, "side_pushpull_after_extrude")) {
        return false;
    }
    if (HasOrphanGeneratedLine(scene, *box)) {
        std::cerr << "side_pushpull: generated boundary line survived outside current object boundary\n";
        return false;
    }

    std::cout << "side_pushpull_drops_old_boundary_lines passed: faces="
              << CountFaces(box) << ", userLines=" << CountUserDrawnLines(scene) << "\n";
    return true;
}

} // namespace

int main()
{
    const bool exactSplitOk = RunInteriorSquareSplitCase("interior_square_on_box_face", 0.0f);
    const bool nearSurfaceSplitOk = RunInteriorSquareSplitCase("near_surface_square_on_box_face", 3.0e-5f);
    const bool moveSplitFaceOk = RunMoveNewSplitFaceCase();
    const bool openSegmentOk = RunOpenSegmentSplitCase();
    const bool overlapNodingOk = RunOverlappingSegmentNodingCase();
    const bool envelopeGraphOk = RunEnvelopeGraphSplitCase();
    const bool partialEnvelopeOk = RunPartialEnvelopeKeepsTopFaceCase();
    const bool envelopePushPullOk = RunEnvelopePushPullPreservesGraphCase();
    const bool sidePushPullOk = RunSidePushPullDropsOldBoundaryLinesCase();

    if (!exactSplitOk || !nearSurfaceSplitOk || !moveSplitFaceOk ||
        !openSegmentOk || !overlapNodingOk || !envelopeGraphOk ||
        !partialEnvelopeOk || !envelopePushPullOk || !sidePushPullOk) {
        return 1;
    }

    std::cout << "FaceSplit smoke tests passed\n";
    return 0;
}
