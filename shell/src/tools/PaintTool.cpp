#include "tools/PaintTool.h"
#include "engine/scene.h"
#include "engine/scene_object.h"
#include "engine/geometry/BRepGeometry.h"
#include "camera.h"
#include "snapping.h"
#include <SDL2/SDL_mouse.h>
#include <limits>
#include <set>
#include <map>
#include <list>
#include <algorithm>
#include <iostream>
#include <glm/gtx/norm.hpp>

namespace { // Anonymous namespace for helpers

// Re-using the helper from SelectTool
std::vector<size_t> FindCoplanarAdjacentTriangles_ForPaint(
    const Urbaxio::Engine::SceneObject& object,
    size_t startTriangleBaseIndex)
{
    const float NORMAL_DOT_TOLERANCE = 0.999f;
    const float PLANE_DIST_TOLERANCE = 1e-4f;
    const auto& mesh = object.getMeshBuffers();
    if (!object.hasMesh() || startTriangleBaseIndex + 2 >= mesh.indices.size()) { return { startTriangleBaseIndex }; }
    std::map<std::pair<unsigned int, unsigned int>, std::vector<size_t>> edgeToTriangles;
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        unsigned int v_indices[3] = { mesh.indices[i], mesh.indices[i + 1], mesh.indices[i + 2] };
        for (int j = 0; j < 3; ++j) {
            unsigned int v1_idx = v_indices[j];
            unsigned int v2_idx = v_indices[(j + 1) % 3];
            if (v1_idx > v2_idx) std::swap(v1_idx, v2_idx);
            edgeToTriangles[{v1_idx, v2_idx}].push_back(i);
        }
    }
    
    std::vector<size_t> resultFaceTriangles;
    std::list<size_t> queue;
    std::set<size_t> visitedTriangles;
    unsigned int i0 = mesh.indices[startTriangleBaseIndex];
    glm::vec3 v0(mesh.vertices[i0*3], mesh.vertices[i0*3+1], mesh.vertices[i0*3+2]);
    glm::vec3 referenceNormal(mesh.normals[i0*3], mesh.normals[i0*3+1], mesh.normals[i0*3+2]);
    float referencePlaneD = -glm::dot(referenceNormal, v0);
    queue.push_back(startTriangleBaseIndex);
    visitedTriangles.insert(startTriangleBaseIndex);
    while (!queue.empty()) {
        size_t currentTriangleIndex = queue.front();
        queue.pop_front();
        resultFaceTriangles.push_back(currentTriangleIndex);
        unsigned int current_v_indices[3] = { mesh.indices[currentTriangleIndex], mesh.indices[currentTriangleIndex + 1], mesh.indices[currentTriangleIndex + 2] };
        for (int j = 0; j < 3; ++j) {
            unsigned int v1_idx = current_v_indices[j];
            unsigned int v2_idx = current_v_indices[(j + 1) % 3];
            if (v1_idx > v2_idx) std::swap(v1_idx, v2_idx);
            const auto& potentialNeighbors = edgeToTriangles.at({v1_idx, v2_idx});
            for (size_t neighborIndex : potentialNeighbors) {
                if (neighborIndex == currentTriangleIndex) continue;
                if (visitedTriangles.find(neighborIndex) == visitedTriangles.end()) {
                    visitedTriangles.insert(neighborIndex);
                    unsigned int n_i0 = mesh.indices[neighborIndex];
                    glm::vec3 n_v0(mesh.vertices[n_i0*3], mesh.vertices[n_i0*3+1], mesh.vertices[n_i0*3+2]);
                    glm::vec3 neighborNormal(mesh.normals[n_i0*3], mesh.normals[n_i0*3+1], mesh.normals[n_i0*3+2]);
                    if (glm::abs(glm::dot(referenceNormal, neighborNormal)) > NORMAL_DOT_TOLERANCE) {
                        float dist = glm::abs(glm::dot(referenceNormal, n_v0) + referencePlaneD);
                        if (dist < PLANE_DIST_TOLERANCE) queue.push_back(neighborIndex);
                    }
                }
            }
        }
    }
    return resultFaceTriangles;
}

}

namespace Urbaxio::Tools {

void PaintTool::Activate(const ToolContext& context) {
    ITool::Activate(context);
    if(context.hoveredObjId) *context.hoveredObjId = 0;
    if(context.hoveredFaceTriangleIndices) context.hoveredFaceTriangleIndices->clear();
}

void PaintTool::Deactivate() {
    if(context.hoveredObjId) *context.hoveredObjId = 0;
    if(context.hoveredFaceTriangleIndices) context.hoveredFaceTriangleIndices->clear();
    ITool::Deactivate();
}

void PaintTool::SetCurrentMaterial(const std::string& materialName) {
    currentMaterialName_ = materialName;
}

void PaintTool::OnLeftMouseDown(int mouseX, int mouseY, bool shift, bool ctrl, const glm::vec3& rayOrigin, const glm::vec3& rayDirection) {
    if (*context.hoveredObjId == 0 || context.hoveredFaceTriangleIndices->empty()) {
        return;
    }
    Urbaxio::Engine::SceneObject* obj = context.scene->get_object_by_id(*context.hoveredObjId);
    if (!obj) return;
    
    // Painting B-Rep objects is not yet supported.
    if (dynamic_cast<Urbaxio::Engine::BRepGeometry*>(obj->getGeometry())) {
        std::cout << "PaintTool: Painting B-Rep objects is not yet implemented." << std::endl;
        return;
    }
    
    // --- NEW LOGIC for MeshGeometry ---
    // The hovered face is defined by the first triangle in the list of co-planar triangles.
    size_t firstTriangleBaseIndex = context.hoveredFaceTriangleIndices->front();

    // Find which mesh group this triangle belongs to.
    // The groups are defined by ranges of indices in the main index buffer.
    for (auto& group : obj->meshGroups) {
        if (firstTriangleBaseIndex >= group.startIndex && firstTriangleBaseIndex < (group.startIndex + group.indexCount)) {
            // Found the group. Change its material name.
            group.materialName = currentMaterialName_;
            std::cout << "PaintTool: Applied material '" << currentMaterialName_ << "' to a mesh group on object " << obj->get_id() << std::endl;

            // This object's material assignment has changed, so the static batch is no longer valid.
            context.scene->MarkStaticGeometryDirty();
            return; // Job done.
        }
    }

    std::cerr << "PaintTool Warning: Could not find a mesh group for the selected face on object " << obj->get_id() << ". This might indicate an issue with mesh group data." << std::endl;
}

void PaintTool::OnUpdate(const SnapResult& snap, const glm::vec3& rayOrigin, const glm::vec3& rayDirection) {
    if (glm::length2(rayDirection) > 1e-9) { // VR path
        updateHover(rayOrigin, rayDirection);
    }
}

void PaintTool::OnMouseMove(int mouseX, int mouseY) {
    // 2D path
    glm::vec3 rayOrigin, rayDir;
    Camera::ScreenToWorldRay(mouseX, mouseY, *context.display_w, *context.display_h, context.camera->GetViewMatrix(), context.camera->GetProjectionMatrix((float)*context.display_w / *context.display_h), rayOrigin, rayDir);
    updateHover(rayOrigin, rayDir);
}

void PaintTool::updateHover(const glm::vec3& rayOrigin, const glm::vec3& rayDirection) {
    *context.hoveredObjId = 0;
    context.hoveredFaceTriangleIndices->clear();
    uint64_t currentHoveredObjId = 0;
    size_t currentHoveredTriangleIdx = 0;
    float closestHitDist = std::numeric_limits<float>::max();
    for (Urbaxio::Engine::SceneObject* obj_ptr : context.scene->get_all_objects()) {
        // --- FIX: Ignore non-exportable objects ---
        if (obj_ptr && obj_ptr->hasMesh() && obj_ptr->isExportable()) {
            const auto& mesh = obj_ptr->getMeshBuffers();
            for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
                glm::vec3 v0(mesh.vertices[mesh.indices[i]*3], mesh.vertices[mesh.indices[i]*3+1], mesh.vertices[mesh.indices[i]*3+2]);
                glm::vec3 v1(mesh.vertices[mesh.indices[i+1]*3], mesh.vertices[mesh.indices[i+1]*3+1], mesh.vertices[mesh.indices[i+1]*3+2]);
                glm::vec3 v2(mesh.vertices[mesh.indices[i+2]*3], mesh.vertices[mesh.indices[i+2]*3+1], mesh.vertices[mesh.indices[i+2]*3+2]);
                float t;
                if (SnappingSystem::RayTriangleIntersect(rayOrigin, rayDirection, v0, v1, v2, t) && t > 0 && t < closestHitDist) {
                    closestHitDist = t;
                    currentHoveredObjId = obj_ptr->get_id();
                    currentHoveredTriangleIdx = i;
                }
            }
        }
    }
    if (currentHoveredObjId != 0) {
        Urbaxio::Engine::SceneObject* hitObject = context.scene->get_object_by_id(currentHoveredObjId);
        if (hitObject) {
            *context.hoveredFaceTriangleIndices = FindCoplanarAdjacentTriangles_ForPaint(*hitObject, currentHoveredTriangleIdx);
            *context.hoveredObjId = currentHoveredObjId;
        }
    }
}

} // namespace Urbaxio::Tools


