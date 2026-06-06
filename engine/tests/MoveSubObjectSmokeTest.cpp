#include "engine/commands/ICommand.h"
#include "engine/geometry/BRepGeometry.h"
#include "engine/scene.h"
#include "engine/scene_object.h"

#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <TopAbs.hxx>
#include <TopExp_Explorer.hxx>

#include <glm/glm.hpp>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace {

bool ValidateShape(Urbaxio::Engine::SceneObject* object, const char* label)
{
    if (!object) {
        std::cerr << label << ": null object\n";
        return false;
    }

    auto* brep = dynamic_cast<Urbaxio::Engine::BRepGeometry*>(object->getGeometry());
    if (!brep || !brep->getShape() || brep->getShape()->IsNull()) {
        std::cerr << label << ": missing B-Rep shape\n";
        return false;
    }

    BRepCheck_Analyzer analyzer(*brep->getShape());
    if (!analyzer.IsValid()) {
        std::cerr << label << ": invalid B-Rep after move\n";
        return false;
    }

    int solidCount = 0;
    int shellCount = 0;
    int faceCount = 0;
    for (TopExp_Explorer ex(*brep->getShape(), TopAbs_SOLID); ex.More(); ex.Next()) ++solidCount;
    for (TopExp_Explorer ex(*brep->getShape(), TopAbs_SHELL); ex.More(); ex.Next()) ++shellCount;
    for (TopExp_Explorer ex(*brep->getShape(), TopAbs_FACE); ex.More(); ex.Next()) ++faceCount;
    std::cout << label << ": solids=" << solidCount << " shells=" << shellCount << " faces=" << faceCount << "\n";
    if (solidCount == 0) {
        std::cerr << label << ": moved shape is not a solid\n";
        return false;
    }

    Bnd_Box box;
    BRepBndLib::Add(*brep->getShape(), box);
    if (!box.IsVoid()) {
        Standard_Real xmin, ymin, zmin, xmax, ymax, zmax;
        box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        std::cout << label << ": bbox=(" << xmin << "," << ymin << "," << zmin << ")-(" << xmax << "," << ymax << "," << zmax << ")\n";
    }

    const auto& mesh = object->getMeshBuffers();
    if (mesh.vertices.empty() || mesh.indices.empty()) {
        std::cerr << label << ": render mesh was not regenerated\n";
        return false;
    }

    return true;
}

bool ValidateBoundaryLines(
    const Urbaxio::Engine::Scene& scene,
    const Urbaxio::Engine::SceneObject* object,
    const char* label)
{
    if (!object || object->boundaryLineIDs.empty()) {
        std::cerr << label << ": object has no boundary lines\n";
        return false;
    }

    const auto& mesh = object->getMeshBuffers();
    const auto& lines = scene.GetAllLines();
    const float tolerance = 2.0e-3f;

    auto pointExistsInMesh = [&](const glm::vec3& point) {
        for (size_t i = 0; i + 2 < mesh.vertices.size(); i += 3) {
            const glm::vec3 meshPoint(mesh.vertices[i], mesh.vertices[i + 1], mesh.vertices[i + 2]);
            if (glm::distance(point, meshPoint) <= tolerance) {
                return true;
            }
        }
        return false;
    };

    for (uint64_t lineId : object->boundaryLineIDs) {
        auto lineIt = lines.find(lineId);
        if (lineIt == lines.end()) {
            std::cerr << label << ": boundary line " << lineId << " is missing from scene lines\n";
            return false;
        }

        const Urbaxio::Engine::Line& line = lineIt->second;
        if (glm::distance(line.start, line.end) <= Urbaxio::SCENE_POINT_EQUALITY_TOLERANCE) {
            std::cerr << label << ": boundary line " << lineId << " collapsed\n";
            return false;
        }

        if (!pointExistsInMesh(line.start) || !pointExistsInMesh(line.end)) {
            std::cerr << label << ": boundary line " << lineId
                      << " endpoint is not on the regenerated mesh\n";
            return false;
        }
    }

    return true;
}

double VolumeOf(Urbaxio::Engine::SceneObject* object)
{
    auto* brep = dynamic_cast<Urbaxio::Engine::BRepGeometry*>(object->getGeometry());
    if (!brep || !brep->getShape()) {
        return 0.0;
    }

    GProp_GProps props;
    BRepGProp::VolumeProperties(*brep->getShape(), props);
    return props.Mass();
}

bool RunMoveCase(
    const char* label,
    Urbaxio::Engine::SubObjectType type,
    const std::vector<glm::vec3>& positions,
    const glm::vec3& translation,
    const double expectedVolume = 0.0)
{
    Urbaxio::Engine::Scene scene;
    Urbaxio::Engine::SceneObject* box = scene.create_box_object(std::string(label) + "_box", 2.0, 2.0, 2.0);
    if (!ValidateShape(box, label)) {
        return false;
    }

    const double beforeVolume = VolumeOf(box);
    scene.RebuildObjectByMovingVertices(box->get_id(), type, positions, translation);

    if (!ValidateShape(box, label)) {
        return false;
    }
    if (!ValidateBoundaryLines(scene, box, label)) {
        return false;
    }

    const double afterVolume = VolumeOf(box);
    if (afterVolume <= 1.0e-8 || beforeVolume <= 1.0e-8) {
        std::cerr << label << ": volume collapsed\n";
        return false;
    }
    if (expectedVolume > 0.0 && std::abs(afterVolume - expectedVolume) > 1.0e-4) {
        std::cerr << label << ": expected volume " << expectedVolume << ", got " << afterVolume << "\n";
        return false;
    }

    std::cout << label << ": volume " << beforeVolume << " -> " << afterVolume << "\n";
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

bool RunPushPullBoundaryLineCase()
{
    Urbaxio::Engine::Scene scene;
    Urbaxio::Engine::SceneObject* box = scene.create_box_object("push_boundary_box", 2.0, 2.0, 2.0);
    if (!ValidateShape(box, "push_boundary_setup")) {
        return false;
    }

    const uint64_t boxId = box->get_id();
    const std::vector<glm::vec3> splitFace = {
        {-0.45f, -0.45f, 1.0f},
        { 0.45f, -0.45f, 1.0f},
        { 0.45f,  0.45f, 1.0f},
        {-0.45f,  0.45f, 1.0f},
    };
    if (!DrawLoop(scene, splitFace)) {
        return false;
    }

    box = scene.get_object_by_id(boxId);
    if (!box || !ValidateShape(box, "push_boundary_after_split")) {
        return false;
    }

    if (!scene.ExtrudeFace(boxId, splitFace, glm::vec3(0.0f, 0.0f, 1.0f), 0.55f, true)) {
        std::cerr << "push_boundary: ExtrudeFace returned false\n";
        return false;
    }

    box = scene.get_object_by_id(boxId);
    if (!box || !ValidateShape(box, "push_boundary_after_extrude")) {
        return false;
    }
    if (!ValidateBoundaryLines(scene, box, "push_boundary_after_extrude")) {
        return false;
    }

    std::cout << "push_boundary_lines passed\n";
    return true;
}

bool RunMultiExtrusionSideFaceMoveCase()
{
    Urbaxio::Engine::Scene scene;
    Urbaxio::Engine::SceneObject* box = scene.create_box_object("multi_extrusion_box", 2.0, 2.0, 2.0);
    if (!ValidateShape(box, "multi_extrusion_setup")) {
        return false;
    }

    const uint64_t boxId = box->get_id();
    const std::vector<glm::vec3> firstIsland = {
        {-0.35f, 0.20f, 1.0f},
        { 0.05f, 0.20f, 1.0f},
        { 0.05f, 0.62f, 1.0f},
        {-0.35f, 0.62f, 1.0f},
    };
    const std::vector<glm::vec3> secondIsland = {
        {0.35f, -0.58f, 1.0f},
        {0.78f, -0.58f, 1.0f},
        {0.78f, -0.18f, 1.0f},
        {0.35f, -0.18f, 1.0f},
    };
    const std::vector<glm::vec3> cornerIsland = {
        {-1.0f, -1.0f, 1.0f},
        {-0.60f, -1.0f, 1.0f},
        {-0.60f, -0.60f, 1.0f},
        {-1.0f, -0.60f, 1.0f},
    };

    auto splitAndExtrude = [&](const std::vector<glm::vec3>& loop, float height) {
        if (!DrawLoop(scene, loop)) {
            return false;
        }
        return scene.ExtrudeFace(boxId, loop, glm::vec3(0.0f, 0.0f, 1.0f), height, true);
    };

    if (!splitAndExtrude(firstIsland, 0.45f) ||
        !splitAndExtrude(secondIsland, 0.35f) ||
        !splitAndExtrude(cornerIsland, 0.55f)) {
        std::cerr << "multi_extrusion: split/extrude setup failed\n";
        return false;
    }

    box = scene.get_object_by_id(boxId);
    if (!box || !ValidateShape(box, "multi_extrusion_after_pushes")) {
        return false;
    }

    const std::vector<glm::vec3> sideFace = {
        {-0.35f, 0.20f, 1.0f},
        { 0.05f, 0.20f, 1.0f},
        { 0.05f, 0.20f, 1.45f},
        {-0.35f, 0.20f, 1.45f},
    };
    if (!scene.RebuildObjectByMovingVertices(
            boxId,
            Urbaxio::Engine::SubObjectType::FACE,
            sideFace,
            glm::vec3(0.0f, -0.12f, -0.06f))) {
        std::cerr << "multi_extrusion: moving side wall returned false\n";
        return false;
    }

    box = scene.get_object_by_id(boxId);
    if (!box || !ValidateShape(box, "multi_extrusion_after_side_move")) {
        return false;
    }
    if (!ValidateBoundaryLines(scene, box, "multi_extrusion_after_side_move")) {
        return false;
    }

    std::cout << "multi_extrusion_side_face_move passed\n";
    return true;
}

bool HasBoundaryLineBetween(
    const Urbaxio::Engine::Scene& scene,
    const Urbaxio::Engine::SceneObject* object,
    const glm::vec3& a,
    const glm::vec3& b,
    float tolerance = 2.0e-3f)
{
    if (!object) {
        return false;
    }

    const auto& lines = scene.GetAllLines();
    for (uint64_t lineId : object->boundaryLineIDs) {
        auto lineIt = lines.find(lineId);
        if (lineIt == lines.end()) {
            continue;
        }

        const auto& line = lineIt->second;
        const bool sameDirection =
            glm::distance(line.start, a) <= tolerance &&
            glm::distance(line.end, b) <= tolerance;
        const bool oppositeDirection =
            glm::distance(line.start, b) <= tolerance &&
            glm::distance(line.end, a) <= tolerance;
        if (sameDirection || oppositeDirection) {
            return true;
        }
    }

    return false;
}

bool RunNonPlanarQuadDiagonalCase()
{
    Urbaxio::Engine::Scene scene;
    Urbaxio::Engine::SceneObject* box = scene.create_box_object("quad_diagonal_box", 2.0, 2.0, 2.0);
    if (!ValidateShape(box, "quad_diagonal_setup")) {
        return false;
    }

    const uint64_t boxId = box->get_id();
    const glm::vec3 p0(-1.0f, -1.0f, 1.0f);
    const glm::vec3 p1( 1.0f, -1.0f, 1.0f);
    const glm::vec3 p2( 1.0f,  1.0f, 1.0f);
    const glm::vec3 p3(-1.0f,  1.0f, 1.0f);
    const glm::vec3 translation(0.0f, 0.50f, -0.40f);
    if (!scene.RebuildObjectByMovingVertices(
            boxId,
            Urbaxio::Engine::SubObjectType::VERTEX,
            {p1},
            translation)) {
        std::cerr << "quad_diagonal: vertex move returned false\n";
        return false;
    }

    box = scene.get_object_by_id(boxId);
    if (!box || !ValidateShape(box, "quad_diagonal_after_move")) {
        return false;
    }

    const glm::vec3 movedP0 = p0;
    const glm::vec3 movedP1 = p1 + translation;
    const glm::vec3 movedP2 = p2;
    const glm::vec3 movedP3 = p3;

    const glm::vec3 d02 = movedP0 - movedP2;
    const glm::vec3 d13 = movedP1 - movedP3;
    const float diag02 = glm::dot(d02, d02);
    const float diag13 = glm::dot(d13, d13);
    const bool expects13 = diag13 < diag02;
    if (expects13) {
        if (!HasBoundaryLineBetween(scene, box, movedP1, movedP3) || HasBoundaryLineBetween(scene, box, movedP0, movedP2)) {
            std::cerr << "quad_diagonal: expected short diagonal p1-p3 only\n";
            return false;
        }
    } else {
        if (!HasBoundaryLineBetween(scene, box, movedP0, movedP2) || HasBoundaryLineBetween(scene, box, movedP1, movedP3)) {
            std::cerr << "quad_diagonal: expected short diagonal p0-p2 only\n";
            return false;
        }
    }

    std::cout << "non_planar_quad_diagonal passed\n";
    return true;
}

} // namespace

int main()
{
    const bool faceMoveOk = RunMoveCase(
        "face_move",
        Urbaxio::Engine::SubObjectType::FACE,
        {
            {-1.0f, -1.0f, 1.0f},
            { 1.0f, -1.0f, 1.0f},
            { 1.0f,  1.0f, 1.0f},
            {-1.0f,  1.0f, 1.0f},
        },
        {0.35f, 0.20f, 0.0f},
        8.0);

    const bool edgeMoveOk = RunMoveCase(
        "edge_move",
        Urbaxio::Engine::SubObjectType::EDGE,
        {
            {-1.0f, -1.0f, -1.0f},
            { 1.0f, -1.0f, -1.0f},
        },
        {0.25f, 0.0f, 0.0f});

    const bool vertexMoveOk = RunMoveCase(
        "vertex_move",
        Urbaxio::Engine::SubObjectType::VERTEX,
        {
            {-1.0f, -1.0f, -1.0f},
        },
        {0.20f, 0.10f, 0.25f});

    const bool pushBoundaryOk = RunPushPullBoundaryLineCase();
    const bool multiExtrusionOk = RunMultiExtrusionSideFaceMoveCase();
    const bool diagonalOk = RunNonPlanarQuadDiagonalCase();

    if (!faceMoveOk || !edgeMoveOk || !vertexMoveOk || !pushBoundaryOk || !multiExtrusionOk || !diagonalOk) {
        return 1;
    }

    std::cout << "MoveSubObject smoke test passed\n";
    return 0;
}
