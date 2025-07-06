#pragma once

#include "snapping.h" // Includes snap result definition
#include <SDL2/SDL_events.h>
#include <SDL2/SDL_video.h>
#include <cstdint>
#include <cstddef>
#include <vector> // For std::vector
#include <glm/glm.hpp>

// Forward declaration
struct ImGuiIO;
namespace Urbaxio { class Camera; namespace Engine { class Scene; } }

namespace Urbaxio {

    class InputHandler {
    public:
        InputHandler();

        void ProcessEvents(
            // Core systems & state
            Urbaxio::Camera& camera,
            bool& should_quit,
            SDL_Window* window,
            int& display_w, int& display_h,
            // Selection state
            uint64_t& selectedObjId,
            std::vector<size_t>& selectedTriangleIndices,
            std::vector<size_t>& selectedLineIndices,
            // Tool state
            bool isDrawingLineMode,
            bool isPushPullMode,
            uint64_t& hoveredObjId,
            std::vector<size_t>& hoveredFaceTriangleIndices,
            // Drawing mode & state (Input/Output)
            bool& isPlacingFirstPoint,
            bool& isPlacingSecondPoint,
            glm::vec3& currentLineStartPoint,
            // Scene & Snapping (Input/Output)
            Urbaxio::Engine::Scene* scene,
            glm::vec3& currentRubberBandEnd,
            SnapResult& currentSnap,
            // Line Length Input State (Input/Output)
            char* lineLengthInputBuf,
            float& lineLengthValue
        );

    private:
        // Input states
        bool middleMouseButtonDown;
        bool shiftDown;
        bool shiftWasPressed;
        int lastMouseX;
        int lastMouseY;
        bool isMouseFocused;
        bool firstMouse;

        // Double-click detection state
        uint32_t lastClickTimestamp;
        uint64_t lastClickedObjId;
        size_t lastClickedTriangleIndex;


        // Axis Locking State
        bool isAxisLocked;
        SnapType lockedAxisType;
        glm::vec3 lockedAxisOrigin;
        glm::vec3 lockedAxisDir;

        // Snapping System instance
        SnappingSystem snappingSystem;

        // Helpers
        void HandleMouseMotion(Urbaxio::Camera& camera, SDL_Window* window, int display_w, int display_h);
        glm::vec3 GetCursorPointInWorld(const Camera& camera, int mouseX, int mouseY, int screenWidth, int screenHeight, const glm::vec3& fallbackPlanePoint);

        // Line picking helper
        bool RayLineSegmentIntersection(
            const glm::vec3& rayOrigin, const glm::vec3& rayDir,
            const glm::vec3& p1, const glm::vec3& p2, // Segment endpoints
            float pickThresholdRadius,                 // How close the ray must pass to the segment
            float& outDistanceAlongRay,             // Output: distance along ray to closest point on ray
            glm::vec3& outClosestPointOnSegment      // Output: closest point on segment to the ray
        );
    };

} // namespace Urbaxio