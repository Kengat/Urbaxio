#include "camera.h"
#include "snapping.h"
#include "engine/scene.h"
#include "engine/scene_object.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/common.hpp>
#include <cmath>
#include <iostream>
#include <limits>

namespace Urbaxio {

    Camera::Camera(glm::vec3 position, glm::vec3 target, glm::vec3 worldUp, float fov, float orbitSpeed, float panSpeed, float zoomSpeed)
        : Position(position), Target(target), WorldUp(worldUp), FovDegrees(fov), MouseSensitivity(orbitSpeed), MovementSpeed(panSpeed), ZoomSensitivity(zoomSpeed), NearPlane(0.1f), FarPlane(1000.0f)
    {
        glm::vec3 radiusVec = Position - Target; Radius = glm::length(radiusVec);
        if (Radius < 1e-6f) { Radius = 1.0f; Position = Target + glm::vec3(0, -Radius, 0); radiusVec = Position - Target; }
        float pitch_arg = radiusVec.z / Radius; Pitch = glm::asin(glm::clamp(pitch_arg, -1.0f, 1.0f)); Yaw = glm::atan(radiusVec.y, radiusVec.x);
        updateCameraVectors();
    }

    glm::mat4 Camera::GetViewMatrix() const { return glm::lookAt(Position, Target, Up); }
    glm::mat4 Camera::GetProjectionMatrix(float aspectRatio) const { if (aspectRatio <= 0.0f) return glm::mat4(1.0f); return glm::perspective(glm::radians(FovDegrees), aspectRatio, NearPlane, FarPlane); }
    void Camera::ProcessOrbit(float xoffset, float yoffset) { Yaw -= xoffset * MouseSensitivity; Pitch += yoffset * MouseSensitivity; Yaw = fmod(Yaw, 2.0f * glm::pi<float>()); updateCameraPositionFromOrbit(); }
    void Camera::ProcessPan(float xoffset, float yoffset, int mouseX, int mouseY, int screenWidth, int screenHeight, const Engine::Scene& scene) {
        // 1. Find the point in the world under the cursor to use as a pivot.
        glm::vec3 rayOrigin, rayDir;
        ScreenToWorldRay(mouseX, mouseY, screenWidth, screenHeight, GetViewMatrix(), GetProjectionMatrix((float)screenWidth / screenHeight), rayOrigin, rayDir);
        
        float closest_t = std::numeric_limits<float>::max();
        bool hit = false;
        
        for (const auto* obj : scene.get_all_objects()) {
            if (obj && obj->has_mesh()) {
                const auto& mesh = obj->get_mesh_buffers();
                for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
                    unsigned int i0 = mesh.indices[i], i1 = mesh.indices[i+1], i2 = mesh.indices[i+2];
                    glm::vec3 v0(mesh.vertices[i0*3], mesh.vertices[i0*3+1], mesh.vertices[i0*3+2]);
                    glm::vec3 v1(mesh.vertices[i1*3], mesh.vertices[i1*3+1], mesh.vertices[i1*3+2]);
                    glm::vec3 v2(mesh.vertices[i2*3], mesh.vertices[i2*3+1], mesh.vertices[i2*3+2]);
                    float t;
                    if (SnappingSystem::RayTriangleIntersect(rayOrigin, rayDir, v0, v1, v2, t) && t > 0 && t < closest_t) {
                        closest_t = t;
                        hit = true;
                    }
                }
            }
        }
        
        glm::vec3 panPivotPoint;
        if (hit) {
            panPivotPoint = rayOrigin + rayDir * closest_t;
        } else {
            glm::vec3 planeNormal = -Front;
            float distanceToPlane = glm::dot(Target - rayOrigin, planeNormal);
            if (distanceToPlane < 0) distanceToPlane = Radius;
            panPivotPoint = rayOrigin + rayDir * distanceToPlane;
        }

        // 2. Calculate the pan speed based on the distance to the pivot point.
        float distance = glm::distance(Position, panPivotPoint);
        float panFactor = distance * 0.001f; // Adjust this factor for sensitivity
        
        glm::vec3 rightMovement = Right * xoffset * panFactor;
        glm::vec3 upMovement = Up * -yoffset * panFactor;

        // 3. Apply the pan offset to both position and target.
        Position -= (rightMovement + upMovement);
        Target -= (rightMovement + upMovement);
        
        updateCameraVectors();
    }
    void Camera::ProcessOrbitZoom(float yoffset) { 
        float zoomFactor = 1.0f - yoffset * (ZoomSensitivity / 10.0f); 
        Radius *= zoomFactor; 
        Radius = glm::clamp(Radius, 0.1f, 1000.0f); 
        updateCameraPositionFromOrbit(); 
    }

    void Camera::ProcessDollyZoom(float yoffset, int mouseX, int mouseY, int screenWidth, int screenHeight, const Engine::Scene& scene) {
        if (screenWidth <= 0 || screenHeight <= 0) return;

        // 1. Find the point in the world under the cursor by raycasting against the scene
        glm::vec3 rayOrigin, rayDir;
        ScreenToWorldRay(mouseX, mouseY, screenWidth, screenHeight, GetViewMatrix(), GetProjectionMatrix((float)screenWidth / screenHeight), rayOrigin, rayDir);
        
        float closest_t = std::numeric_limits<float>::max();
        bool hit = false;
        
        for (const auto* obj : scene.get_all_objects()) {
            if (obj && obj->has_mesh()) {
                const auto& mesh = obj->get_mesh_buffers();
                for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
                    unsigned int i0 = mesh.indices[i], i1 = mesh.indices[i+1], i2 = mesh.indices[i+2];
                    glm::vec3 v0(mesh.vertices[i0*3], mesh.vertices[i0*3+1], mesh.vertices[i0*3+2]);
                    glm::vec3 v1(mesh.vertices[i1*3], mesh.vertices[i1*3+1], mesh.vertices[i1*3+2]);
                    glm::vec3 v2(mesh.vertices[i2*3], mesh.vertices[i2*3+1], mesh.vertices[i2*3+2]);
                    float t;
                    if (SnappingSystem::RayTriangleIntersect(rayOrigin, rayDir, v0, v1, v2, t) && t > 0 && t < closest_t) {
                        closest_t = t;
                        hit = true;
                    }
                }
            }
        }
        
        glm::vec3 pointUnderMouse;
        if (hit) {
            pointUnderMouse = rayOrigin + rayDir * closest_t;
        } else {
            // Fallback: intersect with a plane parallel to the view plane, passing through the current target.
            glm::vec3 planeNormal = -Front;
            float distanceToPlane = glm::dot(Target - rayOrigin, planeNormal);
            // If the target is behind the camera, use a default distance.
            if (distanceToPlane < 0) distanceToPlane = 20.0f;
            pointUnderMouse = rayOrigin + rayDir * distanceToPlane;
        }

        // 2. Calculate the distance to the point and apply scaling with clamping
        float distanceToTarget = glm::distance(Position, pointUnderMouse);
        const float minZoomScaleDist = 1.0f; // NEW: Reduced minimum
        const float maxZoomScaleDist = 500.0f; // NEW: Increased maximum
        float clampedDistance = glm::clamp(distanceToTarget, minZoomScaleDist, maxZoomScaleDist);
        float zoomAmount = yoffset * ZoomSensitivity * (clampedDistance * 0.07f); // NEW: Increased factor

        // 3. Calculate the movement vector and prevent zooming past the target
        glm::vec3 moveVec = glm::normalize(pointUnderMouse - Position) * zoomAmount;
        if (zoomAmount > 0 && glm::length(moveVec) > (distanceToTarget - NearPlane)) {
            moveVec = glm::normalize(pointUnderMouse - Position) * (distanceToTarget - NearPlane);
        }
        
        // 4. Move both the camera position and the target to achieve a "dolly" or "fly" effect
        Position += moveVec;
        Target += moveVec;
        
        // 5. Update derived camera properties
        Radius = glm::distance(Position, Target); // Radius is now relative to the new target
        updateCameraVectors();
    }

    void Camera::SetOrbitTarget(const glm::vec3& hitPoint) {
        // --- NEW LOGIC: The user's elegant solution ---
        // 1. Calculate the distance from the camera to the point under the cursor.
        float distance = glm::distance(Position, hitPoint);
        // 2. Set the new target to be a point this distance away, directly in front of the camera.
        Target = Position + Front * distance;
        // 3. Update the orbit radius.
        Radius = distance;

        // No need to update vectors here, as the camera's orientation (Front) hasn't changed.
    }

    void Camera::UpdateFromPositionTarget() {
        // This logic is copied from the constructor to re-initialize the camera's
        // orbital parameters (Radius, Pitch, Yaw) and orientation vectors (Front, Up, Right)
        // after manually setting Position and Target (e.g., from a file load).
        glm::vec3 radiusVec = Position - Target; Radius = glm::length(radiusVec);
        if (Radius < 1e-6f) { Radius = 1.0f; Position = Target + glm::vec3(0, -Radius, 0); radiusVec = Position - Target; }
        float pitch_arg = radiusVec.z / Radius; Pitch = glm::asin(glm::clamp(pitch_arg, -1.0f, 1.0f)); Yaw = glm::atan(radiusVec.y, radiusVec.x);
        updateCameraVectors();
    }

    void Camera::updateCameraVectors() { Front = glm::normalize(Target - Position); Right = glm::normalize(glm::cross(Front, WorldUp)); Up = glm::normalize(glm::cross(Right, Front)); }
    void Camera::updateCameraPositionFromOrbit() { Position.x = Target.x + Radius * cos(Pitch) * cos(Yaw); Position.y = Target.y + Radius * cos(Pitch) * sin(Yaw); Position.z = Target.z + Radius * sin(Pitch); updateCameraVectors(); }


    // --- Picking Ray Implementation ---
    void Camera::ScreenToWorldRay(
        int mouseX, int mouseY,
        int screenWidth, int screenHeight,
        const glm::mat4& viewMatrix,
        const glm::mat4& projectionMatrix,
        glm::vec3& outRayOrigin,
        glm::vec3& outRayDirection)
    {
        if (screenWidth <= 0 || screenHeight <= 0) return;

        // 1. Normalize Device Coordinates (NDC) [-1, 1]
        //    Invert Y because screen coords usually start top-left, NDC bottom-left
        float x = (2.0f * mouseX) / screenWidth - 1.0f;
        float y = 1.0f - (2.0f * mouseY) / screenHeight;
        float z = -1.0f; // Ray starts at the near plane in NDC
        glm::vec3 ray_nds(x, y, z);

        // 2. Homogeneous Clip Coordinates (Perspective division needed later)
        //    We want a point on the near plane (z=-1) and far plane (z=1)
        //    For perspective projection, w component is -z_eye, so we set w=1 for simplicity here.
        //    But for ray direction, it's simpler to unproject two points.
        glm::vec4 ray_clip_near(ray_nds.x, ray_nds.y, -1.0, 1.0); // Point on near plane
        glm::vec4 ray_clip_far(ray_nds.x, ray_nds.y, 1.0, 1.0);   // Point on far plane

        // 3. Eye (Camera) Coordinates
        //    Inverse projection matrix transforms clip space to eye space
        glm::mat4 InvProjection = glm::inverse(projectionMatrix);
        glm::vec4 ray_eye_near = InvProjection * ray_clip_near;
        glm::vec4 ray_eye_far = InvProjection * ray_clip_far;

        // Perspective division (divide by w)
        if (ray_eye_near.w != 0.0f) ray_eye_near /= ray_eye_near.w; else ray_eye_near.w = 1.0f;
        if (ray_eye_far.w != 0.0f) ray_eye_far /= ray_eye_far.w; else ray_eye_far.w = 1.0f;

        // 4. World Coordinates
        //    Inverse view matrix transforms eye space to world space
        glm::mat4 InvView = glm::inverse(viewMatrix);
        glm::vec4 ray_world_near_h = InvView * ray_eye_near;
        glm::vec4 ray_world_far_h = InvView * ray_eye_far;

        glm::vec3 ray_world_near(ray_world_near_h);
        glm::vec3 ray_world_far(ray_world_far_h);

        // 5. Calculate Ray Origin and Direction
        outRayOrigin = ray_world_near; // Origin is the point on the near plane in world space
        outRayDirection = glm::normalize(ray_world_far - ray_world_near); // Direction is vector from near to far point
    }


} // namespace Urbaxio