#pragma once

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/constants.hpp> // For glm::pi
#include <glm/gtc/quaternion.hpp> // Optional: For more advanced rotations later

namespace Urbaxio {
    // FORWARD DECLARATION: Tell the compiler that the Scene class exists without including the full header.
    namespace Engine { class Scene; }

    // Represents the 3D camera and handles its movement logic.
    class Camera {
    public:
        // Camera Attributes
        glm::vec3 Position;
        glm::vec3 Front;
        glm::vec3 Up;
        glm::vec3 Right;
        glm::vec3 WorldUp;

        // Euler Angles (for orbit calculation)
        float Yaw;   // Angle in the XY plane (around WorldUp)
        float Pitch; // Angle relative to the XY plane

        // Orbit Target
        glm::vec3 Target;
        float Radius; // Distance from Position to Target

        // Camera options
        float MovementSpeed; // Base speed for panning adjusted by radius
        float MouseSensitivity; // Speed for orbit rotation
        float ZoomSensitivity;  // Speed for zooming
        float FovDegrees;       // Field of View in degrees
        float NearPlane;
        float FarPlane;

        // Constructor with vectors
        Camera(glm::vec3 position = glm::vec3(40.0f, 30.0f, 30.0f),
            glm::vec3 target = glm::vec3(0.0f, 0.0f, 0.0f),
            glm::vec3 worldUp = glm::vec3(0.0f, 0.0f, 1.0f), // Z is up
            float fov = 45.0f,
            float orbitSpeed = 0.005f,
            float panSpeed = 0.05f,
            float zoomSpeed = 1.0f);

        // Returns the view matrix calculated using the LookAt method
        glm::mat4 GetViewMatrix() const;

        // Returns the projection matrix
        glm::mat4 GetProjectionMatrix(float aspectRatio) const;

        // Processes input received from mouse movement for orbiting
        void ProcessOrbit(float xoffset, float yoffset);

        // Processes input received from mouse movement for panning
        void ProcessPan(float xoffset, float yoffset, int mouseX, int mouseY, int screenWidth, int screenHeight, const Engine::Scene& scene);

        // Processes input for orbital zoom (zooming towards the target)
        void ProcessOrbitZoom(float yoffset);

        // NEW: Processes input for dolly zoom (zooming towards the mouse cursor)
        void ProcessDollyZoom(float yoffset, int mouseX, int mouseY, int screenWidth, int screenHeight, const Engine::Scene& scene);

        // NEW: Sets a new pivot point for orbiting and re-calculates orbital parameters
        void SetOrbitTarget(const glm::vec3& hitPoint);

        // --- Picking Ray Function ---
        // Calculates a world space ray from screen coordinates
        static void ScreenToWorldRay(
            int mouseX, int mouseY,
            int screenWidth, int screenHeight,
            const glm::mat4& viewMatrix,
            const glm::mat4& projectionMatrix,
            glm::vec3& outRayOrigin,
            glm::vec3& outRayDirection
        );


    private:
        // Calculates the front vector from the Camera's (updated) Euler Angles
        void updateCameraVectors();
        // Updates the camera position based on the current Yaw, Pitch, Radius, and Target
        void updateCameraPositionFromOrbit();
    };

} // namespace Urbaxio