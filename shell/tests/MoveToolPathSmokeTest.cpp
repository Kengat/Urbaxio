#include "tools/MoveTool.h"
#include "engine/commands/MoveCommand.h"
#include "engine/geometry/BRepGeometry.h"
#include "engine/line.h"
#include "engine/scene.h"
#include "engine/scene_object.h"

#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopAbs.hxx>
#include <TopExp_Explorer.hxx>

#include <glm/glm.hpp>

#include <cstdint>
#include <cmath>
#include <iostream>
#include <set>
#include <string>
#include <vector>

namespace {

bool ValidateBRepSolid(Urbaxio::Engine::SceneObject* object, const char* label)
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

    if (!BRepCheck_Analyzer(*brep->getShape()).IsValid()) {
        std::cerr << label << ": invalid B-Rep shape\n";
        return false;
    }

    int solidCount = 0;
    for (TopExp_Explorer ex(*brep->getShape(), TopAbs_SOLID); ex.More(); ex.Next()) {
        ++solidCount;
    }

    if (solidCount == 0) {
        std::cerr << label << ": result has no solid\n";
        return false;
    }

    if (!object->hasMesh() || object->getMeshBuffers().vertices.empty() || object->getMeshBuffers().indices.empty()) {
        std::cerr << label << ": render mesh was not regenerated\n";
        return false;
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

Urbaxio::SnapResult EndpointSnap(uint64_t objectId, const glm::vec3& point)
{
    Urbaxio::SnapResult snap;
    snap.snapped = true;
    snap.type = Urbaxio::SnapType::ENDPOINT;
    snap.worldPoint = point;
    snap.snappedEntityId = objectId;
    return snap;
}

Urbaxio::SnapResult PointSnap(Urbaxio::SnapType type, uint64_t entityId, const glm::vec3& point)
{
    Urbaxio::SnapResult snap;
    snap.snapped = true;
    snap.type = type;
    snap.worldPoint = point;
    snap.snappedEntityId = entityId;
    return snap;
}

struct MoveToolHarness {
    Urbaxio::Engine::Scene scene;
    uint64_t selectedObjId = 0;
    std::vector<size_t> selectedTriangles;
    std::set<uint64_t> selectedLineIds;
    uint64_t hoveredObjId = 0;
    std::vector<size_t> hoveredTriangles;
    bool shiftDown = false;
    bool ctrlDown = false;
    bool isNumpadActive = false;
    float rightThumbstickY = 0.0f;
    float rightTriggerValue = 0.0f;
    int displayW = 1280;
    int displayH = 720;

    Urbaxio::Tools::ToolContext Context()
    {
        Urbaxio::Tools::ToolContext context;
        context.scene = &scene;
        context.display_w = &displayW;
        context.display_h = &displayH;
        context.selectedObjId = &selectedObjId;
        context.selectedTriangleIndices = &selectedTriangles;
        context.selectedLineIDs = &selectedLineIds;
        context.hoveredObjId = &hoveredObjId;
        context.hoveredFaceTriangleIndices = &hoveredTriangles;
        context.shiftDown = &shiftDown;
        context.ctrlDown = &ctrlDown;
        context.isNumpadActive = &isNumpadActive;
        context.rightThumbstickY = &rightThumbstickY;
        context.rightTriggerValue = &rightTriggerValue;
        return context;
    }
};

Urbaxio::Engine::SceneObject* CreateTranslatedBox(MoveToolHarness& harness, const std::string& name, const glm::vec3& offset)
{
    Urbaxio::Engine::SceneObject* box = harness.scene.create_box_object(name, 2.0, 2.0, 2.0);
    if (!box) {
        return nullptr;
    }

    Urbaxio::Engine::MoveCommand moveCommand(&harness.scene, box->get_id(), offset);
    moveCommand.Execute();
    return box;
}

bool NearPoint(const glm::vec3& a, const glm::vec3& b, float tolerance = 1.0e-3f)
{
    return glm::length(a - b) <= tolerance;
}

uint64_t FindLineByEndpoints(const Urbaxio::Engine::Scene& scene, const Urbaxio::Engine::SceneObject& object, const glm::vec3& a, const glm::vec3& b)
{
    const auto& lines = scene.GetAllLines();
    for (uint64_t lineId : object.boundaryLineIDs) {
        auto it = lines.find(lineId);
        if (it == lines.end()) {
            continue;
        }

        const auto& line = it->second;
        if ((NearPoint(line.start, a) && NearPoint(line.end, b)) ||
            (NearPoint(line.start, b) && NearPoint(line.end, a))) {
            return lineId;
        }
    }

    return 0;
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

size_t FindTopFaceTriangle(const Urbaxio::Engine::SceneObject& object, float topZ)
{
    const auto& mesh = object.getMeshBuffers();
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        unsigned int indices[3] = { mesh.indices[i], mesh.indices[i + 1], mesh.indices[i + 2] };
        bool allTop = true;
        for (unsigned int idx : indices) {
            if (static_cast<size_t>(idx) * 3 + 2 >= mesh.vertices.size() ||
                std::abs(mesh.vertices[idx * 3 + 2] - topZ) > 1.0e-3f) {
                allTop = false;
                break;
            }
        }
        if (allTop) {
            return i;
        }
    }

    return static_cast<size_t>(-1);
}

size_t FindTriangleByNormal(const Urbaxio::Engine::SceneObject& object, const glm::vec3& desiredNormal)
{
    const auto& mesh = object.getMeshBuffers();
    const glm::vec3 target = glm::normalize(desiredNormal);
    float bestDot = -1.0f;
    size_t bestTriangle = static_cast<size_t>(-1);

    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        unsigned int i0 = mesh.indices[i + 0];
        unsigned int i1 = mesh.indices[i + 1];
        unsigned int i2 = mesh.indices[i + 2];
        if (static_cast<size_t>(i0) * 3 + 2 >= mesh.vertices.size() ||
            static_cast<size_t>(i1) * 3 + 2 >= mesh.vertices.size() ||
            static_cast<size_t>(i2) * 3 + 2 >= mesh.vertices.size()) {
            continue;
        }

        glm::vec3 v0(mesh.vertices[i0 * 3 + 0], mesh.vertices[i0 * 3 + 1], mesh.vertices[i0 * 3 + 2]);
        glm::vec3 v1(mesh.vertices[i1 * 3 + 0], mesh.vertices[i1 * 3 + 1], mesh.vertices[i1 * 3 + 2]);
        glm::vec3 v2(mesh.vertices[i2 * 3 + 0], mesh.vertices[i2 * 3 + 1], mesh.vertices[i2 * 3 + 2]);
        glm::vec3 normal = glm::cross(v1 - v0, v2 - v0);
        if (glm::length(normal) < 1.0e-6f) {
            continue;
        }

        normal = glm::normalize(normal);
        const float dot = glm::dot(normal, target);
        if (dot > bestDot) {
            bestDot = dot;
            bestTriangle = i;
        }
    }

    return bestDot > 0.55f ? bestTriangle : static_cast<size_t>(-1);
}

bool MoveSelectedFaceOnce(
    MoveToolHarness& harness,
    Urbaxio::Engine::SceneObject* box,
    size_t triangleBaseIndex,
    const glm::vec3& basePoint,
    const glm::vec3& translation,
    const char* label)
{
    harness.selectedObjId = box->get_id();
    harness.selectedTriangles.clear();
    harness.selectedTriangles.push_back(triangleBaseIndex);

    const double beforeVolume = VolumeOf(box);
    Urbaxio::Tools::MoveTool tool;
    tool.Activate(harness.Context());
    tool.OnUpdate(PointSnap(Urbaxio::SnapType::ON_FACE, box->get_id(), basePoint));
    tool.OnLeftMouseDown(0, 0, false, false);

    if (!tool.IsMoving() || tool.GetMovingObjectId() != box->get_id() || !tool.GetGhostMesh()) {
        std::cerr << label << ": MoveTool did not enter face preview\n";
        return false;
    }

    tool.OnUpdate(PointSnap(Urbaxio::SnapType::ON_FACE, box->get_id(), basePoint + translation));
    tool.OnLeftMouseDown(0, 0, false, false);

    if (tool.IsMoving()) {
        std::cerr << label << ": MoveTool did not finalize face move\n";
        return false;
    }

    if (!ValidateBRepSolid(box, label)) {
        return false;
    }

    const double afterVolume = VolumeOf(box);
    if (beforeVolume <= 1.0e-8 || afterVolume <= 1.0e-8) {
        std::cerr << label << ": volume collapsed " << beforeVolume << " -> " << afterVolume << "\n";
        return false;
    }

    std::cout << label << " passed: volume " << beforeVolume << " -> " << afterVolume << "\n";
    return true;
}

bool RunVertexMovePath()
{
    MoveToolHarness harness;
    const glm::vec3 offset(100000.0f, -75000.0f, 25000.0f);
    Urbaxio::Engine::SceneObject* box = CreateTranslatedBox(harness, "move_tool_vertex_far_box", offset);
    if (!ValidateBRepSolid(box, "vertex_before")) {
        return false;
    }

    const double beforeVolume = VolumeOf(box);

    Urbaxio::Tools::MoveTool tool;
    tool.Activate(harness.Context());

    const glm::vec3 startPoint = offset + glm::vec3(-1.0f, -1.0f, -1.0f);
    const glm::vec3 translation(0.20f, 0.10f, 0.25f);
    tool.OnUpdate(EndpointSnap(box->get_id(), startPoint));
    tool.OnLeftMouseDown(0, 0, false, false);

    if (!tool.IsMoving() || tool.GetMovingObjectId() != box->get_id() || !tool.GetGhostMesh()) {
        std::cerr << "MoveTool did not enter vertex preview from ENDPOINT snap\n";
        return false;
    }

    tool.OnUpdate(EndpointSnap(box->get_id(), startPoint + translation));
    tool.OnLeftMouseDown(0, 0, false, false);

    if (tool.IsMoving()) {
        std::cerr << "MoveTool did not finalize vertex move\n";
        return false;
    }

    if (!ValidateBRepSolid(box, "vertex_after")) {
        return false;
    }

    double afterVolume = VolumeOf(box);
    if (beforeVolume <= 1.0e-8 || afterVolume <= 1.0e-8 || afterVolume >= beforeVolume) {
        std::cerr << "Unexpected volume change: " << beforeVolume << " -> " << afterVolume << "\n";
        return false;
    }

    std::cout << "MoveTool vertex path smoke test passed: volume " << beforeVolume << " -> " << afterVolume << "\n";

    const double beforeSecondVolume = afterVolume;
    const glm::vec3 secondStartPoint = startPoint + translation;
    const glm::vec3 secondTranslation(0.05f, 0.05f, 0.05f);
    Urbaxio::Tools::MoveTool secondTool;
    secondTool.Activate(harness.Context());
    secondTool.OnUpdate(EndpointSnap(box->get_id(), secondStartPoint));
    secondTool.OnLeftMouseDown(0, 0, false, false);

    if (!secondTool.IsMoving() || secondTool.GetMovingObjectId() != box->get_id() || !secondTool.GetGhostMesh()) {
        std::cerr << "MoveTool did not enter preview for already-moved vertex\n";
        return false;
    }

    secondTool.OnUpdate(EndpointSnap(box->get_id(), secondStartPoint + secondTranslation));
    secondTool.OnLeftMouseDown(0, 0, false, false);

    if (secondTool.IsMoving()) {
        std::cerr << "MoveTool did not finalize repeated vertex move\n";
        return false;
    }

    if (!ValidateBRepSolid(box, "vertex_after_second_move")) {
        return false;
    }

    afterVolume = VolumeOf(box);
    if (afterVolume <= 1.0e-8 || afterVolume >= beforeSecondVolume) {
        std::cerr << "Unexpected repeated vertex volume change: " << beforeSecondVolume << " -> " << afterVolume << "\n";
        return false;
    }

    std::cout << "MoveTool repeated vertex path smoke test passed: volume " << beforeSecondVolume << " -> " << afterVolume << "\n";
    return true;
}

bool RunEdgeMovePath()
{
    MoveToolHarness harness;
    const glm::vec3 offset(-120000.0f, 80000.0f, 5000.0f);
    Urbaxio::Engine::SceneObject* box = CreateTranslatedBox(harness, "move_tool_edge_far_box", offset);
    if (!ValidateBRepSolid(box, "edge_before")) {
        return false;
    }

    const glm::vec3 a = offset + glm::vec3(-1.0f, -1.0f, -1.0f);
    const glm::vec3 b = offset + glm::vec3(1.0f, -1.0f, -1.0f);
    const uint64_t lineId = FindLineByEndpoints(harness.scene, *box, a, b);
    if (lineId == 0) {
        std::cerr << "Could not find boundary line for edge move\n";
        return false;
    }

    const double beforeVolume = VolumeOf(box);
    const glm::vec3 midpoint = (a + b) * 0.5f;
    const glm::vec3 translation(0.0f, 0.25f, 0.0f);

    Urbaxio::Tools::MoveTool tool;
    tool.Activate(harness.Context());
    tool.OnUpdate(PointSnap(Urbaxio::SnapType::MIDPOINT, lineId, midpoint));
    tool.OnLeftMouseDown(0, 0, false, false);

    if (!tool.IsMoving() || tool.GetMovingObjectId() != box->get_id() || !tool.GetGhostMesh()) {
        std::cerr << "MoveTool did not enter edge preview from MIDPOINT snap\n";
        return false;
    }

    tool.OnUpdate(PointSnap(Urbaxio::SnapType::MIDPOINT, lineId, midpoint + translation));
    tool.OnLeftMouseDown(0, 0, false, false);

    if (tool.IsMoving()) {
        std::cerr << "MoveTool did not finalize edge move\n";
        return false;
    }

    if (!ValidateBRepSolid(box, "edge_after")) {
        return false;
    }

    const double afterVolume = VolumeOf(box);
    if (beforeVolume <= 1.0e-8 || afterVolume <= 1.0e-8 || afterVolume >= beforeVolume) {
        std::cerr << "Unexpected edge volume change: " << beforeVolume << " -> " << afterVolume << "\n";
        return false;
    }

    std::cout << "MoveTool edge path smoke test passed: volume " << beforeVolume << " -> " << afterVolume << "\n";
    return true;
}

bool RunFaceMovePath()
{
    MoveToolHarness harness;
    const glm::vec3 offset(60000.0f, 90000.0f, -30000.0f);
    Urbaxio::Engine::SceneObject* box = CreateTranslatedBox(harness, "move_tool_face_far_box", offset);
    if (!ValidateBRepSolid(box, "face_before")) {
        return false;
    }

    const size_t topTriangle = FindTopFaceTriangle(*box, offset.z + 1.0f);
    if (topTriangle == static_cast<size_t>(-1)) {
        std::cerr << "Could not find top face render triangle\n";
        return false;
    }

    harness.selectedObjId = box->get_id();
    harness.selectedTriangles.push_back(topTriangle);

    const double beforeVolume = VolumeOf(box);
    const glm::vec3 basePoint = offset + glm::vec3(0.0f, 0.0f, 1.0f);
    const glm::vec3 translation(0.35f, 0.20f, 0.0f);

    Urbaxio::Tools::MoveTool tool;
    tool.Activate(harness.Context());
    tool.OnUpdate(PointSnap(Urbaxio::SnapType::ON_FACE, box->get_id(), basePoint));
    tool.OnLeftMouseDown(0, 0, false, false);

    if (!tool.IsMoving() || tool.GetMovingObjectId() != box->get_id() || !tool.GetGhostMesh()) {
        std::cerr << "MoveTool did not enter face preview from selected face\n";
        return false;
    }

    tool.OnUpdate(PointSnap(Urbaxio::SnapType::ON_FACE, box->get_id(), basePoint + translation));
    tool.OnLeftMouseDown(0, 0, false, false);

    if (tool.IsMoving()) {
        std::cerr << "MoveTool did not finalize face move\n";
        return false;
    }

    if (!ValidateBRepSolid(box, "face_after")) {
        return false;
    }

    const double afterVolume = VolumeOf(box);
    if (std::abs(afterVolume - beforeVolume) > 1.0e-4) {
        std::cerr << "Unexpected face volume change: " << beforeVolume << " -> " << afterVolume << "\n";
        return false;
    }

    std::cout << "MoveTool face path smoke test passed: volume " << beforeVolume << " -> " << afterVolume << "\n";
    return true;
}

bool RunRepeatedDifferentFaceMovesPath()
{
    MoveToolHarness harness;
    const glm::vec3 offset(15000.0f, -22000.0f, 18000.0f);
    Urbaxio::Engine::SceneObject* box = CreateTranslatedBox(harness, "move_tool_repeated_faces_box", offset);
    if (!ValidateBRepSolid(box, "repeated_faces_before")) {
        return false;
    }

    size_t tri = FindTriangleByNormal(*box, glm::vec3(0.0f, 0.0f, 1.0f));
    if (tri == static_cast<size_t>(-1) ||
        !MoveSelectedFaceOnce(harness, box, tri, offset + glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.30f, 0.16f, 0.0f), "repeated_face_move_1")) {
        return false;
    }

    tri = FindTriangleByNormal(*box, glm::vec3(1.0f, 0.0f, 0.0f));
    if (tri == static_cast<size_t>(-1) ||
        !MoveSelectedFaceOnce(harness, box, tri, offset + glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.26f, 0.12f), "repeated_face_move_2")) {
        return false;
    }

    tri = FindTriangleByNormal(*box, glm::vec3(0.0f, 1.0f, 0.0f));
    if (tri == static_cast<size_t>(-1) ||
        !MoveSelectedFaceOnce(harness, box, tri, offset + glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.18f, 0.0f, -0.14f), "repeated_face_move_3")) {
        return false;
    }

    std::cout << "MoveTool repeated different face path smoke test passed\n";
    return true;
}

bool RunComplexPreviewTopologyMovePath()
{
    MoveToolHarness harness;
    Urbaxio::Engine::SceneObject* box = harness.scene.create_box_object("move_tool_complex_preview_box", 2.0, 2.0, 2.0);
    if (!ValidateBRepSolid(box, "complex_preview_before")) {
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
        if (!DrawLoop(harness.scene, loop)) {
            return false;
        }
        return harness.scene.ExtrudeFace(boxId, loop, glm::vec3(0.0f, 0.0f, 1.0f), height, true);
    };

    if (!splitAndExtrude(firstIsland, 0.45f) ||
        !splitAndExtrude(secondIsland, 0.35f) ||
        !splitAndExtrude(cornerIsland, 0.55f)) {
        std::cerr << "complex_preview: split/extrude setup failed\n";
        return false;
    }

    box = harness.scene.get_object_by_id(boxId);
    if (!ValidateBRepSolid(box, "complex_preview_after_pushes")) {
        return false;
    }

    size_t sideTriangle = FindTriangleByNormal(*box, glm::vec3(0.0f, -1.0f, 0.0f));
    if (sideTriangle == static_cast<size_t>(-1)) {
        sideTriangle = FindTriangleByNormal(*box, glm::vec3(1.0f, 0.0f, 0.0f));
    }
    if (sideTriangle == static_cast<size_t>(-1)) {
        std::cerr << "complex_preview: could not find side triangle\n";
        return false;
    }

    if (!MoveSelectedFaceOnce(
            harness,
            box,
            sideTriangle,
            glm::vec3(0.0f, -1.0f, 1.1f),
            glm::vec3(0.12f, -0.18f, -0.08f),
            "complex_preview_face_move")) {
        return false;
    }

    box = harness.scene.get_object_by_id(boxId);
    if (!ValidateBRepSolid(box, "complex_preview_after_move")) {
        return false;
    }

    std::cout << "MoveTool complex preview topology path smoke test passed\n";
    return true;
}

} // namespace

int main()
{
    if (!RunVertexMovePath() ||
        !RunEdgeMovePath() ||
        !RunFaceMovePath() ||
        !RunRepeatedDifferentFaceMovesPath() ||
        !RunComplexPreviewTopologyMovePath()) {
        return 1;
    }

    std::cout << "MoveTool path smoke tests passed\n";
    return 0;
}
