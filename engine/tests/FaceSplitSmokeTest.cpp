#include "engine/geometry/BRepGeometry.h"
#include "engine/scene.h"
#include "engine/scene_object.h"

#include <BRepCheck_Analyzer.hxx>
#include <TopAbs.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Shape.hxx>

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

} // namespace

int main()
{
    const bool exactSplitOk = RunInteriorSquareSplitCase("interior_square_on_box_face", 0.0f);
    const bool nearSurfaceSplitOk = RunInteriorSquareSplitCase("near_surface_square_on_box_face", 3.0e-5f);
    const bool moveSplitFaceOk = RunMoveNewSplitFaceCase();

    if (!exactSplitOk || !nearSurfaceSplitOk || !moveSplitFaceOk) {
        return 1;
    }

    std::cout << "FaceSplit smoke tests passed\n";
    return 0;
}
