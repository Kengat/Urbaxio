#include "snapping.h"
#include "camera.h"
#include <engine/scene.h>
#include <engine/scene_object.h>
#include <cad_kernel/MeshBuffers.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/geometric.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/vector_query.hpp>
#include <limits>
#include <cmath>
#include <iostream>
#include <vector>
#include <algorithm>
#include <utility>
#include <set>

namespace { // Anonymous namespace for utility functions
    const float LINE_RAY_EPSILON = 1e-6f;
    const float SCREEN_EPSILON = 1e-4f;
    const float DEPTH_TOLERANCE = 1e-4f; // Epsilon for depth comparison
    
    // --- NEW: Ray-AABB intersection test ---
    bool RayAABBIntersect(
        const glm::vec3& rayOrigin, const glm::vec3& rayDir,
        const glm::vec3& boxMin, const glm::vec3& boxMax,
        float& t_intersect)
    {
        glm::vec3 invDir = 1.0f / rayDir;
        glm::vec3 t0s = (boxMin - rayOrigin) * invDir;
        glm::vec3 t1s = (boxMax - rayOrigin) * invDir;
        glm::vec3 tmin = glm::min(t0s, t1s);
        glm::vec3 tmax = glm::max(t0s, t1s);
        float t_enter = std::max({tmin.x, tmin.y, tmin.z});
        float t_exit = std::min({tmax.x, tmax.y, tmax.z});
        if (t_enter > t_exit || t_exit < 0) {
            return false;
        }
        t_intersect = t_enter;
        return true;
    }
    
    // Calculates the closest points between a ray and a line segment in 3D space.
    bool ClosestPointsRaySegment(
        const glm::vec3& rayOrigin, const glm::vec3& rayDir,
        const glm::vec3& segStart, const glm::vec3& segEnd,
        glm::vec3& outPointOnRay,
        glm::vec3& outPointOnSegment
    ) {
        glm::vec3 segDir = segEnd - segStart;
        
        glm::vec3 w = rayOrigin - segStart;
        float a = glm::dot(rayDir, rayDir);      // Should be 1.0 if rayDir is normalized
        float b = glm::dot(rayDir, segDir);
        float c = glm::dot(segDir, segDir);      // Same as segLenSq
        float d = glm::dot(rayDir, w);
        float e = glm::dot(segDir, w);

        float denom = a * c - b * b;
        float t_ray, t_seg;

        // If the lines are parallel or nearly parallel
        if (std::abs(denom) < LINE_RAY_EPSILON) {
            t_ray = 0.0f; // Any point on the ray is fine, let's use the origin
            t_seg = -e / c; // Project ray origin onto the segment's line
        } else {
            t_ray = (b * e - c * d) / denom;
            t_seg = (a * e - b * d) / denom;
        }

        // Clamp the segment parameter to the range [0, 1] to stay on the segment
        t_seg = std::max(0.0f, std::min(1.0f, t_seg));

        outPointOnRay = rayOrigin + t_ray * rayDir;
        outPointOnSegment = segStart + t_seg * segDir;

        return true;
    }

    glm::vec3 ClosestPointOnLine(const glm::vec3& lineOrigin, const glm::vec3& lineDir, const glm::vec3& point) { float t = glm::dot(point - lineOrigin, lineDir); return lineOrigin + lineDir * t; }
    // --- FIX: Restore the missing helper function ---
    bool ClosestPointLineLine( const glm::vec3& o1, const glm::vec3& d1, const glm::vec3& o2, const glm::vec3& d2, glm::vec3& outPointOnL1) {
        // --- НОВАЯ, БОЛЕЕ НАДЕЖНАЯ РЕАЛИЗАЦИЯ ---
        glm::vec3   v = o1 - o2;
        float    d1d2 = glm::dot(d1, d2);
        float    d2v  = glm::dot(d2, v);
        float    d1v  = glm::dot(d1, v);
        float    d1d1 = glm::dot(d1, d1); // Should be 1.0 if normalized
        float    d2d2 = glm::dot(d2, d2); // Should be 1.0 if normalized

        float denom = d1d1 * d2d2 - d1d2 * d1d2;

        // If lines are parallel, project origin of line 2 onto line 1
        if (std::abs(denom) < LINE_RAY_EPSILON) {
            outPointOnL1 = ClosestPointOnLine(o1, d1, o2);
            return true;
        }

        float t = (d1d2 * d2v - d2d2 * d1v) / denom;
        outPointOnL1 = o1 + t * d1;
        return true;
    }
    bool WorldToScreen(const glm::vec3& worldPos, const glm::mat4& viewMatrix, const glm::mat4& projectionMatrix, int screenWidth, int screenHeight, glm::vec2& outScreenPos) { if (screenWidth <= 0 || screenHeight <= 0) return false; glm::vec4 clipPos = projectionMatrix * viewMatrix * glm::vec4(worldPos, 1.0f); if (clipPos.w <= SCREEN_EPSILON) { return false; } glm::vec3 ndcPos = glm::vec3(clipPos) / clipPos.w; outScreenPos.x = (ndcPos.x + 1.0f) * 0.5f * static_cast<float>(screenWidth); outScreenPos.y = (1.0f - ndcPos.y) * 0.5f * static_cast<float>(screenHeight); return true; }
    bool RaycastToZPlane(int mouseX, int mouseY, int screenWidth, int screenHeight, const Urbaxio::Camera& camera, glm::vec3& outIntersectionPoint) { glm::vec3 rayOrigin, rayDirection; glm::mat4 view = camera.GetViewMatrix(); glm::mat4 projection = camera.GetProjectionMatrix((screenHeight > 0) ? ((float)screenWidth / (float)screenHeight) : 1.0f); Urbaxio::Camera::ScreenToWorldRay(mouseX, mouseY, screenWidth, screenHeight, view, projection, rayOrigin, rayDirection); glm::vec3 planeNormal(0.0f, 0.0f, 1.0f); glm::vec3 pointOnPlane(0.0f, 0.0f, 0.0f); float dirDotNormal = glm::dot(rayDirection, planeNormal); if (std::abs(dirDotNormal) < LINE_RAY_EPSILON) { return false; } float t = glm::dot(pointOnPlane - rayOrigin, planeNormal) / dirDotNormal; if (t < 0.0f) { return false; } outIntersectionPoint = rayOrigin + rayDirection * t; return true; }
    struct SnapCandidate : public Urbaxio::SnapResult { float screenDistSq = std::numeric_limits<float>::max(); };
    
    int GetSnapPriority(Urbaxio::SnapType type) {
        switch (type) {
            // HIGHEST PRIORITY: Specific, tangible geometry points
            case Urbaxio::SnapType::ENDPOINT:       return 100;
            case Urbaxio::SnapType::MIDPOINT:       return 90;
            case Urbaxio::SnapType::INTERSECTION:   return 85;
            
            // HIGH PRIORITY: Tangible geometry lines/surfaces
            case Urbaxio::SnapType::ON_EDGE:        return 80;
            case Urbaxio::SnapType::ON_FACE:        return 70;

            // MEDIUM PRIORITY: Abstract construction geometry
            case Urbaxio::SnapType::ORIGIN:         return 50;
            case Urbaxio::SnapType::CENTER:         return 45;
            case Urbaxio::SnapType::AXIS_X:         return 40;
            case Urbaxio::SnapType::AXIS_Y:         return 40;
            case Urbaxio::SnapType::AXIS_Z:         return 40;
            
            // LOWEST PRIORITY
            case Urbaxio::SnapType::GRID:           return 10;
            case Urbaxio::SnapType::NONE:
            default:                                return 0;
        }
    }
} // end anonymous namespace


namespace Urbaxio {

    SnappingSystem::SnappingSystem() {}
    bool SnappingSystem::WorldToScreen(const glm::vec3& worldPos, const glm::mat4& viewMatrix, const glm::mat4& projectionMatrix, int screenWidth, int screenHeight, glm::vec2& outScreenPos) { return ::WorldToScreen(worldPos, viewMatrix, projectionMatrix, screenWidth, screenHeight, outScreenPos); }
    bool SnappingSystem::RaycastToZPlane(int mouseX, int mouseY, int screenWidth, int screenHeight, const Urbaxio::Camera& camera, glm::vec3& outIntersectionPoint) { return ::RaycastToZPlane(mouseX, mouseY, screenWidth, screenHeight, camera, outIntersectionPoint); }

    bool SnappingSystem::RayTriangleIntersect(
        const glm::vec3& rayOrigin, const glm::vec3& rayDirection,
        const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2,
        float& t_intersection
    ) {
        const float EPSILON_INTERSECT = 1e-6f;
        glm::vec3 edge1 = v1 - v0;
        glm::vec3 edge2 = v2 - v0;
        glm::vec3 h = glm::cross(rayDirection, edge2);
        float a = glm::dot(edge1, h);
        if (a > -EPSILON_INTERSECT && a < EPSILON_INTERSECT) return false;
        float f = 1.0f / a;
        glm::vec3 s = rayOrigin - v0;
        float u = f * glm::dot(s, h);
        if (u < 0.0f || u > 1.0f) return false;
        glm::vec3 q = glm::cross(s, edge1);
        float v = f * glm::dot(rayDirection, q);
        if (v < 0.0f || u + v > 1.0f) return false;
        t_intersection = f * glm::dot(edge2, q);
        if (t_intersection > EPSILON_INTERSECT) return true;
        else return false;
    }


SnapResult SnappingSystem::FindSnapPoint(
    int mouseX, int mouseY,
    int screenWidth, int screenHeight,
    const Camera& camera,
    const Engine::Scene& scene,
    float snapThresholdPixels)
{
    SnapResult finalSnap;
    finalSnap.snapped = false;
    finalSnap.type = SnapType::NONE;
    float pointThresholdSq = snapThresholdPixels * snapThresholdPixels;
    float lineSnapThresholdSq = (snapThresholdPixels * 0.8f) * (snapThresholdPixels * 0.8f);
    if (lineSnapThresholdSq < 1.0f) lineSnapThresholdSq = 1.0f;

    glm::mat4 view = camera.GetViewMatrix();
    glm::mat4 proj = camera.GetProjectionMatrix((screenHeight > 0) ? ((float)screenWidth / (float)screenHeight) : 1.0f);
    glm::vec2 mousePosScreen(static_cast<float>(mouseX), static_cast<float>(mouseY));
    
    glm::vec3 rayOrigin, rayDirection; 
    Camera::ScreenToWorldRay(mouseX, mouseY, screenWidth, screenHeight, view, proj, rayOrigin, rayDirection);
    
    glm::vec3 pointOnPlane; 
    bool planeHit = RaycastToZPlane(mouseX, mouseY, screenWidth, screenHeight, camera, pointOnPlane);
    finalSnap.worldPoint = planeHit ? pointOnPlane : camera.Position + rayDirection * 10.0f;

    std::vector<SnapCandidate> candidates;
    // --- STEP 1: BROAD-PHASE OCCLUSION PASS ---
    std::vector<std::pair<float, const Engine::SceneObject*>> intersectedObjects;
    for (const auto* obj : scene.get_all_objects()) {
        if (obj && obj->aabbValid) {
            float t;
            if (RayAABBIntersect(rayOrigin, rayDirection, obj->aabbMin, obj->aabbMax, t)) {
                intersectedObjects.push_back({t, obj});
            }
        }
    }
    std::sort(intersectedObjects.begin(), intersectedObjects.end(), [](const auto& a, const auto& b){ return a.first < b.first; });
    
    float min_hit_distance = std::numeric_limits<float>::max();
    SnapCandidate faceSnapCandidate;
    bool faceWasHit = false;
    // --- STEP 2: NARROW-PHASE OCCLUSION & GATHERING ---
    for (const auto& [dist, obj] : intersectedObjects) {
        if (dist > min_hit_distance) break; // No need to check objects that are further away
        if (!obj->has_mesh()) continue;
        
        const auto& name = obj->get_name();
        if (name == "CenterMarker" || name == "UnitCapsuleMarker10m" || name == "UnitCapsuleMarker5m" || name == "LeftControllerVisual" || name == "RightControllerVisual") continue;
        
        // --- NARROW-PHASE: Per-triangle hit test for occlusion ---
        const auto& mesh = obj->get_mesh_buffers();
        for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
            unsigned int i0 = mesh.indices[i], i1 = mesh.indices[i+1], i2 = mesh.indices[i+2];
            glm::vec3 v0(mesh.vertices[i0*3], mesh.vertices[i0*3+1], mesh.vertices[i0*3+2]);
            glm::vec3 v1(mesh.vertices[i1*3], mesh.vertices[i1*3+1], mesh.vertices[i1*3+2]);
            glm::vec3 v2(mesh.vertices[i2*3], mesh.vertices[i2*3+1], mesh.vertices[i2*3+2]);
            float t;
            if (RayTriangleIntersect(rayOrigin, rayDirection, v0, v1, v2, t) && t > 0 && t < min_hit_distance) {
                min_hit_distance = t;
                faceSnapCandidate.snapped = true;
                faceSnapCandidate.type = SnapType::ON_FACE;
                faceSnapCandidate.worldPoint = rayOrigin + rayDirection * t;
                faceSnapCandidate.snappedEntityId = obj->get_id();
                faceSnapCandidate.snappedTriangleIndex = i;
                faceWasHit = true;
            }
        }
    }
    if (faceWasHit) {
        glm::vec2 screenPos;
        if(WorldToScreen(faceSnapCandidate.worldPoint, view, proj, screenWidth, screenHeight, screenPos)) {
            faceSnapCandidate.screenDistSq = glm::length2(mousePosScreen - screenPos);
            candidates.push_back(faceSnapCandidate);
        }
    }

    // --- STEP 3: GATHER OTHER CANDIDATES (Lines, Vertices, Axes) ---
    // Only check lines/vertices of intersected objects + free lines
    
    std::set<uint64_t> processedObjects;
    for (const auto& [dist, obj] : intersectedObjects) {
        if (obj) processedObjects.insert(obj->get_id());
    }

    // Collect ignored line IDs from controller visuals
    std::vector<const Engine::SceneObject*> objects = scene.get_all_objects();
    std::set<uint64_t> ignoredLineIDs;
    for (const auto* obj : objects) {
        if (obj) {
            const auto& name = obj->get_name();
            if (name == "LeftControllerVisual" || name == "RightControllerVisual") {
                ignoredLineIDs.insert(obj->boundaryLineIDs.begin(), obj->boundaryLineIDs.end());
            }
        }
    }

    // Origin
    glm::vec2 originScreenPos;
    if (WorldToScreen(glm::vec3(0.0f), view, proj, screenWidth, screenHeight, originScreenPos)) {
        float dist_along_ray = glm::dot(glm::vec3(0.0f) - rayOrigin, rayDirection);
        if (dist_along_ray < min_hit_distance + DEPTH_TOLERANCE) {
            float distSq = glm::length2(mousePosScreen - originScreenPos);
            if (distSq < pointThresholdSq) {
                candidates.push_back({ {true, glm::vec3(0.0f), SnapType::ORIGIN, 0}, distSq });
            }
        }
    }

    // User Lines
    const auto& lines = scene.GetAllLines();
    for (const auto& [lineId, line] : lines) {
        if (ignoredLineIDs.count(lineId)) { continue; }
        // Endpoints
        for (const auto& endpoint : {line.start, line.end}) {
            float dist_along_ray = glm::dot(endpoint - rayOrigin, rayDirection);
            if (dist_along_ray < min_hit_distance + DEPTH_TOLERANCE) {
                glm::vec2 scrPos;
                if (WorldToScreen(endpoint, view, proj, screenWidth, screenHeight, scrPos)) {
                    float dSq = glm::length2(mousePosScreen - scrPos);
                    if (dSq < pointThresholdSq) { candidates.push_back({{true, endpoint, SnapType::ENDPOINT, lineId}, dSq}); }
                }
            }
        }
        // Midpoint
        glm::vec3 midpoint = (line.start + line.end) * 0.5f;
        float dist_along_ray_mid = glm::dot(midpoint - rayOrigin, rayDirection);
        if (dist_along_ray_mid < min_hit_distance + DEPTH_TOLERANCE) {
            glm::vec2 scrPosMid;
            if (WorldToScreen(midpoint, view, proj, screenWidth, screenHeight, scrPosMid)) {
                float dSq = glm::length2(mousePosScreen - scrPosMid);
                if (dSq < pointThresholdSq) { candidates.push_back({{true, midpoint, SnapType::MIDPOINT, lineId}, dSq}); }
            }
        }
        // On Edge
        glm::vec3 closestPointOnRay, closestPointOnSegment;
        if (ClosestPointsRaySegment(rayOrigin, rayDirection, line.start, line.end, closestPointOnRay, closestPointOnSegment)) {
             float dist_along_ray_edge = glm::dot(closestPointOnSegment - rayOrigin, rayDirection);
             if (dist_along_ray_edge < min_hit_distance + DEPTH_TOLERANCE) {
                glm::vec2 screenSnapPos;
                if (WorldToScreen(closestPointOnSegment, view, proj, screenWidth, screenHeight, screenSnapPos)) {
                    float distSqToScreen = glm::length2(mousePosScreen - screenSnapPos);
                    if (distSqToScreen < lineSnapThresholdSq) {
                        candidates.push_back({ {true, closestPointOnSegment, SnapType::ON_EDGE, lineId}, distSqToScreen });
                    }
                }
            }
        }
    }

    // Object Vertices
    for (const auto* obj : objects) {
        if (processedObjects.count(obj->get_id())) {
            if (obj && obj->has_mesh()) {
                const auto& name = obj->get_name();
                if (name == "CenterMarker" || name == "UnitCapsuleMarker10m" || name == "UnitCapsuleMarker5m" || name == "LeftControllerVisual" || name == "RightControllerVisual") continue;
                
                const auto& vertices_obj = obj->get_mesh_buffers().vertices;
                for (size_t i = 0; i < vertices_obj.size(); i += 3) {
                    glm::vec3 vertexPos(vertices_obj[i], vertices_obj[i+1], vertices_obj[i+2]);
                    float dist_along_ray = glm::dot(vertexPos - rayOrigin, rayDirection);
                    if (dist_along_ray < min_hit_distance + DEPTH_TOLERANCE) {
                        glm::vec2 scrPos;
                        if (WorldToScreen(vertexPos, view, proj, screenWidth, screenHeight, scrPos)) {
                            float dSq = glm::length2(mousePosScreen - scrPos);
                            if (dSq < pointThresholdSq) { candidates.push_back({{true, vertexPos, SnapType::ENDPOINT, obj->get_id()}, dSq}); }
                        }
                    }
                }
            }
        }
    }

    // Axes
    struct AxisInfo { glm::vec3 origin; glm::vec3 dir; SnapType type; };
    std::vector<AxisInfo> axes = {
        {glm::vec3(0.0f), glm::normalize(glm::vec3(1.0f, 0.0f, 0.0f)), SnapType::AXIS_X},
        {glm::vec3(0.0f), glm::normalize(glm::vec3(0.0f, 1.0f, 0.0f)), SnapType::AXIS_Y},
        {glm::vec3(0.0f), glm::normalize(glm::vec3(0.0f, 0.0f, 1.0f)), SnapType::AXIS_Z}
    };
    for (const auto& axis : axes) {
        glm::vec3 pointOnAxis3D;
        // --- FIX: Use ClosestPointLineLine for infinite axes ---
        if (ClosestPointLineLine(axis.origin, axis.dir, rayOrigin, rayDirection, pointOnAxis3D)) {
            // Check if axis point is in front of geometry
            float dist_along_ray = glm::dot(pointOnAxis3D - rayOrigin, rayDirection);
            if (dist_along_ray > 0 && dist_along_ray < min_hit_distance + DEPTH_TOLERANCE) {
                glm::vec2 axisPointScreenPos;
                if (WorldToScreen(pointOnAxis3D, view, proj, screenWidth, screenHeight, axisPointScreenPos)) {
                    float distSqScreen = glm::length2(mousePosScreen - axisPointScreenPos);
                    if (distSqScreen < lineSnapThresholdSq) {
                        candidates.push_back({ {true, pointOnAxis3D, axis.type, 0}, distSqScreen });
                    }
                }
            }
        }
    }
    
    // --- FINAL STEP: SELECT BEST CANDIDATE ---
    if (!candidates.empty()) {
        std::sort(candidates.begin(), candidates.end(), [](const SnapCandidate& a, const SnapCandidate& b) {
            int priorityA = GetSnapPriority(a.type);
            int priorityB = GetSnapPriority(b.type);
            if (priorityA != priorityB) return priorityA > priorityB;
            return a.screenDistSq < b.screenDistSq;
        });
        finalSnap = candidates[0];
    }
    return finalSnap;
}

// VR ray-based snapping
SnapResult SnappingSystem::FindSnapPointFromRay(
    const glm::vec3& rayOrigin, const glm::vec3& rayDirection,
    const Engine::Scene& scene,
    float pickThresholdRadius)
{
    struct VRSnapCandidate : public SnapResult { float distanceAlongRay = std::numeric_limits<float>::max(); };
    SnapResult finalSnap; finalSnap.snapped = false; finalSnap.type = SnapType::NONE;
    float pickThresholdRadiusSq = pickThresholdRadius * pickThresholdRadius;
    
    // Step 1: Broad-phase to find candidate objects
    std::vector<std::pair<float, const Engine::SceneObject*>> intersectedObjects;
    for (const auto* obj : scene.get_all_objects()) {
        if (obj && obj->aabbValid) {
            float t;
            if (RayAABBIntersect(rayOrigin, rayDirection, obj->aabbMin, obj->aabbMax, t)) {
                intersectedObjects.push_back({t, obj});
            }
        }
    }
    std::sort(intersectedObjects.begin(), intersectedObjects.end(), [](const auto& a, const auto& b){ return a.first < b.first; });
    
    // Step 2: Narrow-phase for occlusion
    float min_hit_distance = std::numeric_limits<float>::max();
    VRSnapCandidate faceSnapCandidate; bool faceWasHit = false;
    std::vector<VRSnapCandidate> candidates;
    
    // Step 0: add z=0 plane as lowest-priority GRID candidate
    {
        glm::vec3 planeNormal(0.0f, 0.0f, 1.0f);
        float denom = glm::dot(rayDirection, planeNormal);
        if (std::abs(denom) > LINE_RAY_EPSILON) {
            float t_plane = glm::dot(glm::vec3(0.0f) - rayOrigin, planeNormal) / denom;
            if (t_plane > 0.0f) {
                glm::vec3 onPlane = rayOrigin + t_plane * rayDirection;
                candidates.push_back({ { true, onPlane, SnapType::GRID, 0 }, t_plane });
            }
        }
    }
    
    for (const auto& [dist, obj] : intersectedObjects) {
        if (dist > min_hit_distance) break;
        if (!obj->has_mesh()) continue;
        
        const auto& name = obj->get_name();
        if (name == "CenterMarker" || name == "UnitCapsuleMarker10m" || name == "UnitCapsuleMarker5m" || name == "LeftControllerVisual" || name == "RightControllerVisual") continue;
        
        const auto& mesh = obj->get_mesh_buffers();
        for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
            unsigned int i0 = mesh.indices[i], i1 = mesh.indices[i+1], i2 = mesh.indices[i+2];
            glm::vec3 v0(mesh.vertices[i0*3], mesh.vertices[i0*3+1], mesh.vertices[i0*3+2]);
            glm::vec3 v1(mesh.vertices[i1*3], mesh.vertices[i1*3+1], mesh.vertices[i1*3+2]);
            glm::vec3 v2(mesh.vertices[i2*3], mesh.vertices[i2*3+1], mesh.vertices[i2*3+2]);
            float t;
            if (RayTriangleIntersect(rayOrigin, rayDirection, v0, v1, v2, t) && t > 0 && t < min_hit_distance) {
                min_hit_distance = t; faceWasHit = true;
                faceSnapCandidate.snapped = true; faceSnapCandidate.type = SnapType::ON_FACE;
                faceSnapCandidate.worldPoint = rayOrigin + rayDirection * t;
                faceSnapCandidate.snappedEntityId = obj->get_id();
                faceSnapCandidate.snappedTriangleIndex = i;
            }
        }
    }
    
    if (faceWasHit) { faceSnapCandidate.distanceAlongRay = min_hit_distance; candidates.push_back(faceSnapCandidate); }
    finalSnap.worldPoint = faceWasHit ? faceSnapCandidate.worldPoint : (rayOrigin + rayDirection * 100.0f);

    // Collect ignored line IDs from controller visuals
    std::vector<const Engine::SceneObject*> objects = scene.get_all_objects();
    std::set<uint64_t> ignoredLineIDs;
    for (const auto* obj : objects) {
        if (obj) {
            const auto& name = obj->get_name();
            if (name == "LeftControllerVisual" || name == "RightControllerVisual") {
                ignoredLineIDs.insert(obj->boundaryLineIDs.begin(), obj->boundaryLineIDs.end());
            }
        }
    }

    auto addPointCandidate = [&](const glm::vec3& p, SnapType type, uint64_t id) {
        float t = glm::dot(p - rayOrigin, rayDirection);
        if (t < 0 || t > min_hit_distance + 1e-4f) return;
        glm::vec3 pointOnRay = rayOrigin + t * rayDirection;
        if (glm::distance2(p, pointOnRay) < pickThresholdRadiusSq) {
            candidates.push_back({ {true, p, type, id}, t });
        }
    };

    // Origin
    addPointCandidate(glm::vec3(0.0f), SnapType::ORIGIN, 0);

    // Lines from scene
    const auto& lines = scene.GetAllLines();
    for (const auto& [lineId, line] : lines) {
        if (ignoredLineIDs.count(lineId)) { continue; }
        addPointCandidate(line.start, SnapType::ENDPOINT, lineId);
        addPointCandidate(line.end,   SnapType::ENDPOINT, lineId);
        glm::vec3 midpoint = (line.start + line.end) * 0.5f;
        addPointCandidate(midpoint, SnapType::MIDPOINT, lineId);

        glm::vec3 p_on_ray, p_on_seg;
        if (ClosestPointsRaySegment(rayOrigin, rayDirection, line.start, line.end, p_on_ray, p_on_seg)) {
            float t = glm::dot(p_on_ray - rayOrigin, rayDirection);
            if (t > 0 && t < min_hit_distance + 1e-4f) {
                if (glm::distance2(p_on_ray, p_on_seg) < pickThresholdRadiusSq) {
                    candidates.push_back({ {true, p_on_seg, SnapType::ON_EDGE, lineId}, t });
                }
            }
        }
    }

    // Axes
    struct AxisInfo { glm::vec3 origin; glm::vec3 dir; SnapType type; };
    std::vector<AxisInfo> axes = {
        {glm::vec3(0.0f), glm::normalize(glm::vec3(1.0f, 0.0f, 0.0f)), SnapType::AXIS_X},
        {glm::vec3(0.0f), glm::normalize(glm::vec3(0.0f, 1.0f, 0.0f)), SnapType::AXIS_Y},
        {glm::vec3(0.0f), glm::normalize(glm::vec3(0.0f, 0.0f, 1.0f)), SnapType::AXIS_Z}
    };
    for (const auto& axis : axes) {
        glm::vec3 p_on_axis; ClosestPointLineLine(axis.origin, axis.dir, rayOrigin, rayDirection, p_on_axis);
        float t = glm::dot(p_on_axis - rayOrigin, rayDirection);
        if (t > 0 && t < min_hit_distance + 1e-4f) {
            glm::vec3 p_on_ray = rayOrigin + t * rayDirection;
            if (glm::distance2(p_on_ray, p_on_axis) < pickThresholdRadiusSq) {
                candidates.push_back({ {true, p_on_axis, axis.type, 0}, t });
            }
        }
    }

    if (!candidates.empty()) {
        std::sort(candidates.begin(), candidates.end(), [](const VRSnapCandidate& a, const VRSnapCandidate& b) {
            int pa = GetSnapPriority(a.type), pb = GetSnapPriority(b.type);
            if (pa != pb) return pa > pb; return a.distanceAlongRay < b.distanceAlongRay; });
        finalSnap = candidates[0];
    }
    return finalSnap;
}

} // namespace Urbaxio