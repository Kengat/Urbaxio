#pragma once

#include <SDL2/SDL_events.h>
#include <SDL2/SDL_video.h>
#include <glm/glm.hpp>
#include <engine/scene.h> // <-- FIX: Include the full definition instead of forward declaring

// Forward declarations
namespace Urbaxio { 
    class Camera;
    // namespace Urbaxio::Engine { class Scene; } // No longer needed
    namespace Tools { class ToolManager; }
}

namespace Urbaxio {

    class InputHandler {
    public:
        // --- NEW: Public member to store dropped file path ---
        std::string droppedFilePath;

        InputHandler();

        void ProcessEvents(
            // Core systems
            Urbaxio::Camera& camera,
            bool& should_quit,
            SDL_Window* window,
            // The tool manager to forward events to
            Tools::ToolManager& toolManager,
            // Scene for global actions like Undo/Redo
            Urbaxio::Engine::Scene* scene
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
        void HandleMouseMotion(Urbaxio::Camera& camera, SDL_Window* window, int display_w, int display_h, const Urbaxio::Engine::Scene& scene);
    };

} // namespace Urbaxio