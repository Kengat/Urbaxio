#include "camera.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/common.hpp>
#include <cmath>
#include <iostream>

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
    void Camera::ProcessPan(float xoffset, float yoffset) { float currentPanSpeed = MovementSpeed * (Radius / 50.0f); currentPanSpeed = glm::clamp(currentPanSpeed, 0.001f, MovementSpeed * 10.0f); glm::vec3 rightMovement = Right * xoffset * currentPanSpeed; glm::vec3 upMovement = Up * (-yoffset) * currentPanSpeed; glm::vec3 panOffset = rightMovement + upMovement; Position -= panOffset; Target -= panOffset; updateCameraVectors(); }
    void Camera::ProcessMouseScroll(float yoffset) { float zoomFactor = 1.0f - yoffset * (ZoomSensitivity / 10.0f); Radius *= zoomFactor; Radius = glm::clamp(Radius, 0.1f, 1000.0f); updateCameraPositionFromOrbit(); }
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