#pragma once

#include <glm/glm.hpp>
#include <SDL2/SDL_keycode.h>
#include <cstdint>
#include <vector>
#include <set>
#include <string>
#include "snapping.h"

// Forward declarations to avoid including heavy headers
namespace Urbaxio {
    class Camera;
    class Renderer;
    namespace Engine {
        class Scene;
    }
}
struct SDL_Window;

namespace Urbaxio::Tools {

// Enum to identify tools
enum class ToolType {
    Select,
    Line,
    PushPull,
    Move,
    Paint
};

// Bundles common objects and state that tools need to access.
// Pointers are used to allow tools to modify the main application state (like selection).
struct ToolContext {
    Engine::Scene* scene = nullptr;
    Camera* camera = nullptr;
    SDL_Window* window = nullptr;
    int* display_w = nullptr;
    int* display_h = nullptr;

    // Selection state references (tools can modify these)
    uint64_t* selectedObjId = nullptr;
    std::vector<size_t>* selectedTriangleIndices = nullptr;
    std::set<uint64_t>* selectedLineIDs = nullptr;

    // Hover state references (tools can modify these)
    uint64_t* hoveredObjId = nullptr;
    std::vector<size_t>* hoveredFaceTriangleIndices = nullptr;

    // Modifier key state references
    bool* shiftDown = nullptr;
    bool* ctrlDown = nullptr;
    bool* isNumpadActive = nullptr;
    float* rightThumbstickY = nullptr;

    // NEW: VR world transform for scale-aware calculations
    const glm::mat4* worldTransform = nullptr;

    // --- START OF MODIFICATION ---
    // NEW: Flag to indicate if running in VR
    bool isVrMode = false;
    // --- END OF MODIFICATION ---
};

// Interface for all interactive tools
class ITool {
public:
    virtual ~ITool() = default;

    virtual ToolType GetType() const = 0;
    virtual const char* GetName() const = 0;

    // Lifecycle
    virtual void Activate(const ToolContext& context) {
        this->context = context;
        isActive = true;
    }
    virtual void Deactivate() {
        isActive = false;
    }

    // Event handling (called from InputHandler)
    // -- START OF MODIFICATION --
    virtual void OnLeftMouseDown(int mouseX, int mouseY, bool shift, bool ctrl, const glm::vec3& rayOrigin = {}, const glm::vec3& rayDirection = {}) {}
    virtual void OnLeftMouseUp(int mouseX, int mouseY, bool shift, bool ctrl) {}
    // -- END OF MODIFICATION --
    virtual void OnRightMouseDown() {}
    virtual void OnMouseMove(int mouseX, int mouseY) {}
    virtual void OnKeyDown(SDL_Keycode key, bool shift, bool ctrl) {}
    
    // Per-frame update, gives the tool the latest snap info and the current pointer ray
    virtual void OnUpdate(const SnapResult& snap, const glm::vec3& rayOrigin = {}, const glm::vec3& rayDirection = {}) {
        lastSnapResult = snap;
    } 

    // Rendering (called from main loop)
    virtual void RenderUI() {}
    virtual void RenderPreview(Renderer& renderer, const SnapResult& snap) {}

    bool IsActive() const { return isActive; }

protected:
    ToolContext context;
    bool isActive = false;
    SnapResult lastSnapResult;
};

} // namespace Urbaxio::Tools 