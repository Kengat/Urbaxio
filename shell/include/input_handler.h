#pragma once

#include "snapping.h" // Includes snap result definition
#include <SDL2/SDL_events.h>
#include <SDL2/SDL_video.h>
#include <cstdint>
#include <cstddef>
#include <vector> // For std::vector
#include <set> // For std::set
#include <map> // For std::map
#include <glm/glm.hpp>

// Forward declaration
struct ImGuiIO;
namespace Urbaxio { class Camera; namespace Engine { class Scene; struct Line; } }

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
            std::set<uint64_t>& selectedLineIDs,
            // Tool state
            bool isDrawingLineMode,
            bool isPushPullMode,
            bool& isPushPullActive,
            uint64_t& hoveredObjId,
            std::vector<size_t>& hoveredFaceTriangleIndices,
            float& pushPullCurrentDistance,
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
        
        // --- Getters for Push/Pull state (needed by main loop for preview) ---
        uint64_t GetPushPullObjectId() const { return pushPull_objId; }
        const std::vector<size_t>& GetPushPullFaceIndices() const { return pushPull_faceIndices; }
        const glm::vec3& GetPushPullNormal() const { return pushPull_faceNormal; }

        // This function is useful outside, so let's make it public
        glm::vec3 GetCursorPointInWorld(const Camera& camera, int mouseX, int mouseY, int screenWidth, int screenHeight, const glm::vec3& fallbackPlanePoint);

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

        // Push/Pull internal state
        uint64_t pushPull_objId;
        std::vector<size_t> pushPull_faceIndices;
        glm::vec3 pushPull_faceNormal;
        glm::vec3 pushPull_startPoint;
        int pushPull_startMouseX;
        int pushPull_startMouseY;

        // Axis Locking State
        bool isAxisLocked;
        SnapType lockedAxisType;
        glm::vec3 lockedAxisOrigin;
        glm::vec3 lockedAxisDir;

        // Snapping System instance
        SnappingSystem snappingSystem;

        // Helpers
        void HandleMouseMotion(Urbaxio::Camera& camera, SDL_Window* window, int display_w, int display_h);

        // Line picking helper
        bool RayLineSegmentIntersection(
            const glm::vec3& rayOrigin, const glm::vec3& rayDir,
            const glm::vec3& p1, const glm::vec3& p2, // Segment endpoints
            float pickThresholdRadius,                 // How close the ray must pass to the segment
            float& outDistanceAlongRay,             // Output: distance along ray to closest point on ray
            glm::vec3& outClosestPointOnSegment,      // Output: closest point on segment to the ray
            const std::map<uint64_t, Engine::Line>& lines,
            uint64_t& outHitLineId
        );
    };

} // namespace Urbaxio