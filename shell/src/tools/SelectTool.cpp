#include "tools/SelectTool.h"
#include "engine/scene.h"
#include "engine/scene_object.h"
#include "cad_kernel/MeshBuffers.h"
#include "camera.h"
#include "snapping.h"
#include <SDL2/SDL.h>
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp> // <-- FIX: Include header for distance2 and length2
#include <iostream>
#include <vector>
#include <set>
#include <list>
#include <map>
#include <algorithm>
#include <limits>

namespace { // Anonymous namespace for helpers local to this file

const float LINE_PICK_THRESHOLD_RADIUS = 0.25f;

// Helper to find all coplanar and adjacent triangles, starting from a given one.
std::vector<size_t> FindCoplanarAdjacentTriangles(
    const Urbaxio::Engine::SceneObject& object,
    size_t startTriangleBaseIndex)
{
    // Do not attempt to group faces on special marker objects
    const auto& name = object.get_name();
    if (name == "CenterMarker" || name == "UnitSphereMarker" || name == "UnitCapsuleMarker") {
        return { startTriangleBaseIndex };
    }

    const float NORMAL_DOT_TOLERANCE = 0.999f; // Cosine of angle tolerance
    const float PLANE_DIST_TOLERANCE = 1e-4f;

    const auto& mesh = object.get_mesh_buffers();
    if (!object.has_mesh() || startTriangleBaseIndex + 2 >= mesh.indices.size()) {
        return { startTriangleBaseIndex };
    }

    // Build an edge-to-triangle map for adjacency lookups
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

    // Define the reference plane from the starting triangle
    unsigned int i0 = mesh.indices[startTriangleBaseIndex];
    glm::vec3 v0(mesh.vertices[i0*3], mesh.vertices[i0*3+1], mesh.vertices[i0*3+2]);
    glm::vec3 referenceNormal(mesh.normals[i0*3], mesh.normals[i0*3+1], mesh.normals[i0*3+2]);
    float referencePlaneD = -glm::dot(referenceNormal, v0);

    // Start BFS
    queue.push_back(startTriangleBaseIndex);
    visitedTriangles.insert(startTriangleBaseIndex);

    while (!queue.empty()) {
        size_t currentTriangleIndex = queue.front();
        queue.pop_front();
        resultFaceTriangles.push_back(currentTriangleIndex);

        unsigned int current_v_indices[3] = { mesh.indices[currentTriangleIndex], mesh.indices[currentTriangleIndex + 1], mesh.indices[currentTriangleIndex + 2] };

        // Check neighbors through all 3 edges
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
                    
                    // Check for coplanarity (normal and distance from plane)
                    if (glm::abs(glm::dot(referenceNormal, neighborNormal)) > NORMAL_DOT_TOLERANCE) {
                        float dist = glm::abs(glm::dot(referenceNormal, n_v0) + referencePlaneD);
                        if (dist < PLANE_DIST_TOLERANCE) {
                            queue.push_back(neighborIndex);
                        }
                    }
                }
            }
        }
    }
    return resultFaceTriangles;
}

// Helper to find the closest line segment to a ray
bool RayLineSegmentIntersection(
    const glm::vec3& rayOrigin, const glm::vec3& rayDir,
    const glm::vec3& p1, const glm::vec3& p2,
    float pickThresholdRadius,
    float& outDistanceAlongRay)
{
    const float LINE_RAY_EPSILON = 1e-6f;
    glm::vec3 segDir = p2 - p1;
    float segLenSq = glm::length2(segDir);

    if (segLenSq < LINE_RAY_EPSILON * LINE_RAY_EPSILON) { // It's a point
        outDistanceAlongRay = glm::dot(p1 - rayOrigin, rayDir);
        if (outDistanceAlongRay < 0) return false;
        glm::vec3 pointOnRay = rayOrigin + rayDir * outDistanceAlongRay;
        return glm::distance2(pointOnRay, p1) < pickThresholdRadius * pickThresholdRadius;
    }
    
    glm::vec3 segDirNormalized = glm::normalize(segDir);
    glm::vec3 w0 = rayOrigin - p1;
    
    float a = 1.0f;
    float b = glm::dot(rayDir, segDirNormalized);
    float c = 1.0f;
    float d = glm::dot(rayDir, w0);
    float e = glm::dot(segDirNormalized, w0);
    
    float denom = a * c - b * b;
    float t_seg_param;
    
    if (std::abs(denom) < LINE_RAY_EPSILON) { // Parallel lines
        return false;
    }
    
    t_seg_param = (a * e - b * d) / denom;
    
    // Clamp to segment
    float segActualLength = glm::sqrt(segLenSq);
    t_seg_param = glm::clamp(t_seg_param, 0.0f, segActualLength);
    
    glm::vec3 closestPointOnSegment = p1 + segDirNormalized * t_seg_param;
    float actual_t_ray = glm::dot(closestPointOnSegment - rayOrigin, rayDir);
    
    if (actual_t_ray < 0) return false;
    
    glm::vec3 closestPointOnRay = rayOrigin + actual_t_ray * rayDir;
    
    if (glm::distance2(closestPointOnRay, closestPointOnSegment) < pickThresholdRadius * pickThresholdRadius) {
        outDistanceAlongRay = actual_t_ray;
        return true;
    }
    return false;
}

} // end anonymous namespace

namespace Urbaxio::Tools {

void SelectTool::Activate(const ToolContext& context) {
    ITool::Activate(context);
    lastClickTimestamp = 0;
    lastClickedObjId = 0;
    lastClickedTriangleIndex = 0;
}

void SelectTool::Deactivate() {
    ITool::Deactivate();
}

void SelectTool::OnLeftMouseDown(int mouseX, int mouseY, bool shift, bool ctrl) {
    if (!context.scene || !context.camera || !context.display_w || !context.display_h) return;

    glm::vec3 rayOrigin, rayDir;
    Camera::ScreenToWorldRay(
        mouseX, mouseY, *context.display_w, *context.display_h,
        context.camera->GetViewMatrix(),
        context.camera->GetProjectionMatrix((float)*context.display_w / (float)*context.display_h),
        rayOrigin, rayDir
    );

    uint64_t hitObjectId = 0;
    size_t hitTriangleBaseIndex = 0;
    float closestHitDistance = std::numeric_limits<float>::max();

    struct LineHit { uint64_t lineId; float distanceAlongRay; };
    std::vector<LineHit> lineHits;

    // 1. Check for line intersections first
    const auto& all_lines = context.scene->GetAllLines();
    for (const auto& [id, line] : all_lines) {
        float distAlongRay;
        if (RayLineSegmentIntersection(rayOrigin, rayDir, line.start, line.end, LINE_PICK_THRESHOLD_RADIUS, distAlongRay)) {
            lineHits.push_back({id, distAlongRay});
        }
    }

    // If a line was hit, select it and we're done.
    if (!lineHits.empty()) {
        std::sort(lineHits.begin(), lineHits.end(), [](const LineHit& a, const LineHit& b) { return a.distanceAlongRay < b.distanceAlongRay; });
        uint64_t closestLineId = lineHits[0].lineId;
        
        if (!shift) {
            context.selectedLineIDs->clear();
        }
        
        if (context.selectedLineIDs->count(closestLineId)) {
            context.selectedLineIDs->erase(closestLineId);
        } else {
            context.selectedLineIDs->insert(closestLineId);
        }
        
        // Clear object selection when selecting lines
        *context.selectedObjId = 0;
        context.selectedTriangleIndices->clear();
        return;
    }

    // 2. If no line was hit, check for object face intersections
    if (!shift) {
        context.selectedLineIDs->clear();
    }

    for (Urbaxio::Engine::SceneObject* obj_ptr : context.scene->get_all_objects()) {
        const auto& name = obj_ptr->get_name();
        if (obj_ptr && obj_ptr->has_mesh() &&
            name != "CenterMarker" && name != "UnitSphereMarker" && name != "UnitCapsuleMarker") {
            const auto& mesh = obj_ptr->get_mesh_buffers();
            for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
                glm::vec3 v0(mesh.vertices[mesh.indices[i] * 3], mesh.vertices[mesh.indices[i] * 3 + 1], mesh.vertices[mesh.indices[i] * 3 + 2]);
                glm::vec3 v1(mesh.vertices[mesh.indices[i + 1] * 3], mesh.vertices[mesh.indices[i + 1] * 3 + 1], mesh.vertices[mesh.indices[i + 1] * 3 + 2]);
                glm::vec3 v2(mesh.vertices[mesh.indices[i + 2] * 3], mesh.vertices[mesh.indices[i + 2] * 3 + 1], mesh.vertices[mesh.indices[i + 2] * 3 + 2]);
                float t;
                if (SnappingSystem::RayTriangleIntersect(rayOrigin, rayDir, v0, v1, v2, t) && t > 0 && t < closestHitDistance) {
                    closestHitDistance = t;
                    hitObjectId = obj_ptr->get_id();
                    hitTriangleBaseIndex = i;
                }
            }
        }
    }

    // 3. Process the face hit
    if (hitObjectId != 0) {
        uint32_t currentTime = SDL_GetTicks();
        const uint32_t DOUBLE_CLICK_TIME = 300;

        // Double-click to select a single triangle
        if (currentTime - lastClickTimestamp < DOUBLE_CLICK_TIME && hitObjectId == lastClickedObjId && hitTriangleBaseIndex == lastClickedTriangleIndex) {
            *context.selectedTriangleIndices = { hitTriangleBaseIndex };
            *context.selectedObjId = hitObjectId;
            lastClickTimestamp = 0; // Reset double-click timer
        } else { // Single-click to select a whole face
            Urbaxio::Engine::SceneObject* hitObject = context.scene->get_object_by_id(hitObjectId);
            if (hitObject) {
                *context.selectedTriangleIndices = FindCoplanarAdjacentTriangles(*hitObject, hitTriangleBaseIndex);
                *context.selectedObjId = hitObjectId;
            }
            lastClickTimestamp = currentTime;
            lastClickedObjId = hitObjectId;
            lastClickedTriangleIndex = hitTriangleBaseIndex;
        }
    } else {
        // Clicked on empty space, deselect everything
        if (!shift) {
            *context.selectedObjId = 0;
            context.selectedTriangleIndices->clear();
        }
        lastClickTimestamp = 0;
    }
}

} // namespace Urbaxio::Tools 