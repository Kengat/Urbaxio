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

namespace { // Anonymous namespace for utility functions
    const float LINE_RAY_EPSILON = 1e-6f;
    const float SCREEN_EPSILON = 1e-4f;
    // ... (ClosestPointOnLine, ClosestPointLineLine, WorldToScreen, ScreenToCameraRay, RaycastToZPlane, SnapCandidate struct, ClosestPointOnScreenSegmentSq)
    glm::vec3 ClosestPointOnLine(const glm::vec3& lineOrigin, const glm::vec3& lineDir, const glm::vec3& point) { float t = glm::dot(point - lineOrigin, lineDir); return lineOrigin + lineDir * t; }
    bool ClosestPointLineLine( const glm::vec3& o1, const glm::vec3& d1, const glm::vec3& o2, const glm::vec3& d2, glm::vec3& outPointOnL1) { glm::vec3 w = o1 - o2; float b = glm::dot(d1, d2); float d_dot = glm::dot(d1, w); float e_dot = glm::dot(d2, w); float denom = 1.0f - b * b; if (std::abs(denom) < LINE_RAY_EPSILON) { outPointOnL1 = ClosestPointOnLine(o1, d1, o2); return true; } float s = (b * e_dot - d_dot) / denom; outPointOnL1 = o1 + s * d1; return true; }
    bool WorldToScreen(const glm::vec3& worldPos, const glm::mat4& viewMatrix, const glm::mat4& projectionMatrix, int screenWidth, int screenHeight, glm::vec2& outScreenPos) { if (screenWidth <= 0 || screenHeight <= 0) return false; glm::vec4 clipPos = projectionMatrix * viewMatrix * glm::vec4(worldPos, 1.0f); if (clipPos.w <= SCREEN_EPSILON) { return false; } glm::vec3 ndcPos = glm::vec3(clipPos) / clipPos.w; outScreenPos.x = (ndcPos.x + 1.0f) * 0.5f * static_cast<float>(screenWidth); outScreenPos.y = (1.0f - ndcPos.y) * 0.5f * static_cast<float>(screenHeight); return true; }
    void ScreenToCameraRay(int screenX, int screenY, int screenWidth, int screenHeight, const glm::mat4& invViewProjMatrix, const glm::vec3& cameraPos, glm::vec3& outRayOrigin, glm::vec3& outRayDirection) { float x_ndc = (2.0f * static_cast<float>(screenX)) / static_cast<float>(screenWidth) - 1.0f; float y_ndc = 1.0f - (2.0f * static_cast<float>(screenY)) / static_cast<float>(screenHeight); glm::vec4 worldNear_h = invViewProjMatrix * glm::vec4(x_ndc, y_ndc, -1.0f, 1.0f); if (std::abs(worldNear_h.w) < LINE_RAY_EPSILON) { worldNear_h.w = LINE_RAY_EPSILON; } glm::vec3 worldNear = glm::vec3(worldNear_h) / worldNear_h.w; outRayOrigin = cameraPos; outRayDirection = glm::normalize(worldNear - cameraPos); }
    bool RaycastToZPlane(int mouseX, int mouseY, int screenWidth, int screenHeight, const Urbaxio::Camera& camera, glm::vec3& outIntersectionPoint) { glm::vec3 rayOrigin, rayDirection; glm::mat4 view = camera.GetViewMatrix(); glm::mat4 projection = camera.GetProjectionMatrix((screenHeight > 0) ? ((float)screenWidth / (float)screenHeight) : 1.0f); Urbaxio::Camera::ScreenToWorldRay(mouseX, mouseY, screenWidth, screenHeight, view, projection, rayOrigin, rayDirection); glm::vec3 planeNormal(0.0f, 0.0f, 1.0f); glm::vec3 pointOnPlane(0.0f, 0.0f, 0.0f); float dirDotNormal = glm::dot(rayDirection, planeNormal); if (std::abs(dirDotNormal) < LINE_RAY_EPSILON) { return false; } float t = glm::dot(pointOnPlane - rayOrigin, planeNormal) / dirDotNormal; if (t < 0.0f) { return false; } outIntersectionPoint = rayOrigin + rayDirection * t; return true; }
    struct SnapCandidate : public Urbaxio::SnapResult { float screenDistSq = std::numeric_limits<float>::max(); };
    float ClosestPointOnScreenSegmentSq(const glm::vec2& p, const glm::vec2& a, const glm::vec2& b, glm::vec2& closest, float& outT) { glm::vec2 ab = b - a; glm::vec2 ap = p - a; float lenSqAB = glm::length2(ab); if (lenSqAB < SCREEN_EPSILON * SCREEN_EPSILON) { closest = a; outT = 0.0f; return glm::length2(ap); } outT = glm::dot(ap, ab) / lenSqAB; outT = std::max(0.0f, std::min(1.0f, outT)); closest = a + outT * ab; return glm::length2(p - closest); }

    int GetSnapPriority(Urbaxio::SnapType type) {
        switch (type) {
            case Urbaxio::SnapType::ENDPOINT:       return 10;
            case Urbaxio::SnapType::ORIGIN:         return 9;
            case Urbaxio::SnapType::MIDPOINT:       return 8;
            case Urbaxio::SnapType::INTERSECTION:   return 7;
            case Urbaxio::SnapType::CENTER:         return 7;
            case Urbaxio::SnapType::ON_EDGE:        return 6;
            case Urbaxio::SnapType::AXIS_X:         return 5;
            case Urbaxio::SnapType::AXIS_Y:         return 5;
            case Urbaxio::SnapType::AXIS_Z:         return 5;
            case Urbaxio::SnapType::ON_FACE:        return 3;
            case Urbaxio::SnapType::GRID:           return 1;
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
        SnapResult finalSnap; finalSnap.snapped = false; finalSnap.type = SnapType::NONE;
        float pointThresholdSq = snapThresholdPixels * snapThresholdPixels;
        float lineSnapThresholdSq = (snapThresholdPixels * 0.8f) * (snapThresholdPixels * 0.8f);
        if (lineSnapThresholdSq < 1.0f) lineSnapThresholdSq = 1.0f;

        glm::mat4 view = camera.GetViewMatrix(); glm::mat4 proj = camera.GetProjectionMatrix((screenHeight > 0) ? ((float)screenWidth / (float)screenHeight) : 1.0f);
        glm::vec2 mousePosScreen(static_cast<float>(mouseX), static_cast<float>(mouseY));
        
        // Calculate ray once for all checks that might need it (Axes, OnFace)
        glm::vec3 rayOrigin, rayDirection; 
        Camera::ScreenToWorldRay(mouseX, mouseY, screenWidth, screenHeight, view, proj, rayOrigin, rayDirection);
        
        glm::vec3 pointOnPlane; bool planeHit = RaycastToZPlane(mouseX, mouseY, screenWidth, screenHeight, camera, pointOnPlane);
        finalSnap.worldPoint = planeHit ? pointOnPlane : camera.Position + rayDirection * 10.0f;

        std::vector<SnapCandidate> candidates;

        // 1. Origin (same)
        glm::vec2 originScreenPos; if (WorldToScreen(glm::vec3(0.0f), view, proj, screenWidth, screenHeight, originScreenPos)) { float distSq = glm::length2(mousePosScreen - originScreenPos); if (distSq < pointThresholdSq) { candidates.push_back({ {true, glm::vec3(0.0f), SnapType::ORIGIN}, distSq }); } }

        // 2. User Line Endpoints, Midpoints, On Edge (same)
        const auto& lineSegments = scene.GetLineSegments();
        for (size_t segIdx = 0; segIdx < lineSegments.size(); ++segIdx) {
            const auto& segment = lineSegments[segIdx];
            const glm::vec3 endpoints[] = {segment.first, segment.second};
            for (const auto& endpoint : endpoints) { glm::vec2 scrPos; if (WorldToScreen(endpoint, view, proj, screenWidth, screenHeight, scrPos)) { float dSq = glm::length2(mousePosScreen - scrPos); if (dSq < pointThresholdSq) { bool add = true; for(const auto&c:candidates){if((c.type==SnapType::ENDPOINT||c.type==SnapType::ORIGIN||c.type==SnapType::MIDPOINT)&&glm::distance2(c.worldPoint,endpoint)<LINE_RAY_EPSILON*LINE_RAY_EPSILON){add=false;break;}} if(add) candidates.push_back({{true,endpoint,SnapType::ENDPOINT},dSq}); } } }
            glm::vec3 midpoint = (segment.first + segment.second) * 0.5f; glm::vec2 scrPosMid; if (WorldToScreen(midpoint, view, proj, screenWidth, screenHeight, scrPosMid)) { float dSq = glm::length2(mousePosScreen - scrPosMid); if (dSq < pointThresholdSq) { bool add = true; for(const auto&c:candidates){if((c.type==SnapType::ENDPOINT||c.type==SnapType::ORIGIN||c.type==SnapType::MIDPOINT)&&glm::distance2(c.worldPoint,midpoint)<LINE_RAY_EPSILON*LINE_RAY_EPSILON){add=false;break;}} if(add) { candidates.push_back({{true,midpoint,SnapType::MIDPOINT},dSq}); }} }
            glm::vec2 screenP1, screenP2; bool p1Visible = WorldToScreen(segment.first, view, proj, screenWidth, screenHeight, screenP1); bool p2Visible = WorldToScreen(segment.second, view, proj, screenWidth, screenHeight, screenP2);
            if (p1Visible && p2Visible) { glm::vec2 closestPtOnScreenSeg; float t_2d; float distSqToScreenSeg = ClosestPointOnScreenSegmentSq(mousePosScreen, screenP1, screenP2, closestPtOnScreenSeg, t_2d); if (distSqToScreenSeg < lineSnapThresholdSq) { glm::vec3 worldPointOnEdge = glm::mix(segment.first, segment.second, t_2d); candidates.push_back({ {true, worldPointOnEdge, SnapType::ON_EDGE}, distSqToScreenSeg }); } }
        }

        // 3. Scene Object Vertices and Faces
        std::vector<const Engine::SceneObject*> objects = scene.get_all_objects();
        for (const auto* obj : objects) {
            if (obj && obj->has_mesh()) {
                const auto& mesh = obj->get_mesh_buffers();
                const auto& vertices_obj = mesh.vertices;
                // Vertex Snapping
                for (size_t i = 0; i < vertices_obj.size(); i += 3) { glm::vec3 vertexPos(vertices_obj[i], vertices_obj[i+1], vertices_obj[i+2]); glm::vec2 scrPos; if (WorldToScreen(vertexPos, view, proj, screenWidth, screenHeight, scrPos)) { float dSq = glm::length2(mousePosScreen - scrPos); if (dSq < pointThresholdSq) { bool add = true; for(const auto&c:candidates){if((c.type==SnapType::ENDPOINT||c.type==SnapType::ORIGIN||c.type==SnapType::MIDPOINT)&&glm::distance2(c.worldPoint,vertexPos)<LINE_RAY_EPSILON*LINE_RAY_EPSILON){add=false;break;}} if(add) candidates.push_back({{true,vertexPos,SnapType::ENDPOINT},dSq}); } } }

                // On Face Snapping
                const auto& indices = mesh.indices;
                float closest_t_face = std::numeric_limits<float>::max();
                glm::vec3 hitPointFace_obj; // Changed name to avoid conflict
                bool faceHit_obj = false;   // Changed name

                for (size_t i = 0; i + 2 < indices.size(); i += 3) {
                    unsigned int i0 = indices[i], i1 = indices[i+1], i2 = indices[i+2];
                    size_t max_v_idx = vertices_obj.size() / 3;
                    if (i0 >= max_v_idx || i1 >= max_v_idx || i2 >= max_v_idx) continue;

                    // <<< FIX: Construct glm::vec3 from individual float components >>>
                    glm::vec3 v0(vertices_obj[i0*3 + 0], vertices_obj[i0*3 + 1], vertices_obj[i0*3 + 2]);
                    glm::vec3 v1(vertices_obj[i1*3 + 0], vertices_obj[i1*3 + 1], vertices_obj[i1*3 + 2]);
                    glm::vec3 v2(vertices_obj[i2*3 + 0], vertices_obj[i2*3 + 1], vertices_obj[i2*3 + 2]);
                    float t_intersect_val; // Renamed to avoid conflict with other 't'
                    if (RayTriangleIntersect(rayOrigin, rayDirection, v0, v1, v2, t_intersect_val)) { // Use member function
                        if (t_intersect_val > 0 && t_intersect_val < closest_t_face) {
                            closest_t_face = t_intersect_val;
                            hitPointFace_obj = rayOrigin + rayDirection * t_intersect_val; // Use correct ray
                            faceHit_obj = true;
                        }
                    }
                }
                if (faceHit_obj) {
                    glm::vec2 screenHitPos;
                    if (WorldToScreen(hitPointFace_obj, view, proj, screenWidth, screenHeight, screenHitPos)) {
                        float screenDistSqForFace = glm::length2(mousePosScreen - screenHitPos);
                        if (screenDistSqForFace < pointThresholdSq * 4.0f ) { // Allow leeway
                           candidates.push_back({{true, hitPointFace_obj, SnapType::ON_FACE}, screenDistSqForFace });
                        }
                    }
                }
            }
        }

        // 4. Check Axis Snaps (X, Y, Z) (same)
        struct AxisInfo { glm::vec3 origin; glm::vec3 dir; SnapType type; }; std::vector<AxisInfo> axes = { {glm::vec3(0.0f), glm::normalize(glm::vec3(1.0f, 0.0f, 0.0f)), SnapType::AXIS_X}, {glm::vec3(0.0f), glm::normalize(glm::vec3(0.0f, 1.0f, 0.0f)), SnapType::AXIS_Y}, {glm::vec3(0.0f), glm::normalize(glm::vec3(0.0f, 0.0f, 1.0f)), SnapType::AXIS_Z} };
        for (const auto& axis : axes) { glm::vec3 pointOnAxis3D; if (ClosestPointLineLine(axis.origin, axis.dir, rayOrigin, rayDirection, pointOnAxis3D)) { glm::vec2 axisPointScreenPos; if (WorldToScreen(pointOnAxis3D, view, proj, screenWidth, screenHeight, axisPointScreenPos)) { float distSqScreen = glm::length2(mousePosScreen - axisPointScreenPos); if (distSqScreen < lineSnapThresholdSq) { candidates.push_back({ {true, pointOnAxis3D, axis.type}, distSqScreen }); } } } }

        // Select the best candidate (same)
        if (!candidates.empty()) { std::sort(candidates.begin(), candidates.end(), [](const SnapCandidate& a, const SnapCandidate& b) { int priorityA = GetSnapPriority(a.type); int priorityB = GetSnapPriority(b.type); if (priorityA != priorityB) { return priorityA > priorityB; } return a.screenDistSq < b.screenDistSq; }); finalSnap = candidates[0]; if (finalSnap.type == SnapType::AXIS_X || finalSnap.type == SnapType::AXIS_Y || finalSnap.type == SnapType::AXIS_Z) { for(const auto& axis : axes) { if(axis.type == finalSnap.type) { finalSnap.worldPoint = ClosestPointOnLine(axis.origin, axis.dir, finalSnap.worldPoint); break; } } } }
        return finalSnap;
    }

} // namespace Urbaxio