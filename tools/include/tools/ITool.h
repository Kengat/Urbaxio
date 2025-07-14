#pragma once

#include <glm/glm.hpp>
#include <SDL2/SDL_keycode.h>
#include <cstdint>
#include <vector>
#include <set>
#include <string>

// Forward declarations for types we only use as pointers
namespace Urbaxio {
    class Camera;
    class Renderer;
    struct SnapResult;
    namespace Engine {
        class Scene;
    }
}
struct SDL_Window;

// Full includes for types used in virtual method signatures
namespace Urbaxio {
    class Renderer;
    struct SnapResult;
}

namespace Urbaxio::Tools {

// Enum to identify tools
enum class ToolType {
    Select,
    Line,
    PushPull
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
    virtual void OnLeftMouseDown(int mouseX, int mouseY, bool shift, bool ctrl) {}
    virtual void OnLeftMouseUp(int mouseX, int mouseY, bool shift, bool ctrl) {}
    virtual void OnRightMouseDown() {}
    virtual void OnMouseMove(int mouseX, int mouseY) {}
    virtual void OnKeyDown(SDL_Keycode key, bool shift, bool ctrl) {}
    
    // Per-frame update, gives the tool the latest snap info
    virtual void OnUpdate(const SnapResult& snap) {} 

    // Rendering (called from main loop)
    virtual void RenderUI() {}
    virtual void RenderPreview(Renderer& renderer, const SnapResult& snap) {}

    bool IsActive() const { return isActive; }

protected:
    ToolContext context;
    bool isActive = false;
};

} // namespace Urbaxio::Tools 