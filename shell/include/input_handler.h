#pragma once

#include <SDL2/SDL_events.h>
#include <SDL2/SDL_video.h>
#include <glm/glm.hpp>

// Forward declarations
namespace Urbaxio { 
    class Camera; 
    namespace Tools { class ToolManager; }
}

namespace Urbaxio {

    class InputHandler {
    public:
        InputHandler();

        void ProcessEvents(
            // Core systems
            Urbaxio::Camera& camera,
            bool& should_quit,
            SDL_Window* window,
            // The tool manager to forward events to
            Tools::ToolManager& toolManager
        );
        
        // This function is useful for getting a fallback cursor position
        glm::vec3 GetCursorPointInWorld(const Camera& camera, int mouseX, int mouseY, int screenWidth, int screenHeight, const glm::vec3& fallbackPlanePoint);

    private:
        // Input states
        bool middleMouseButtonDown;
        bool shiftDown;
        bool ctrlDown;
        int lastMouseX;
        int lastMouseY;
        bool isMouseFocused;
        bool firstMouse;

        // Helpers
        void HandleMouseMotion(Urbaxio::Camera& camera, SDL_Window* window, int display_w, int display_h);
    };

} // namespace Urbaxio