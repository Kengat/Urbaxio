#include "tools/ToolManager.h"
#include "tools/SelectTool.h"
#include "tools/LineTool.h"
#include "tools/PushPullTool.h"
#include "tools/MoveTool.h" // <-- NEW

namespace Urbaxio::Tools {

ToolManager::ToolManager(const ToolContext& context) : context(context) {
    tools[ToolType::Select] = std::make_unique<SelectTool>();
    tools[ToolType::Line] = std::make_unique<LineTool>();
    tools[ToolType::PushPull] = std::make_unique<PushPullTool>();
    tools[ToolType::Move] = std::make_unique<MoveTool>(); // <-- NEW

    // Start with the Select tool active
    SetTool(ToolType::Select);
}

void ToolManager::SetTool(ToolType type) {
    if (activeTool && activeTool->GetType() == type) {
        return; // Tool is already active
    }

    if (activeTool) {
        activeTool->Deactivate();
    }

    auto it = tools.find(type);
    if (it != tools.end()) {
        activeTool = it->second.get();
        activeTool->Activate(context);
    } else {
        activeTool = nullptr;
    }
}

ITool* ToolManager::GetActiveTool() const {
    return activeTool;
}

ToolType ToolManager::GetActiveToolType() const {
    return activeTool ? activeTool->GetType() : ToolType::Select;
}

bool ToolManager::ShouldEnableSnapping() const {
    if (!activeTool) return true; // Default to enabled
    
    ToolType type = activeTool->GetType();
    
    // Disable snapping for SelectTool
    if (type == ToolType::Select) {
        return false;
    }
    
    // For PushPullTool, disable snapping only when hovering (before operation starts)
    if (type == ToolType::PushPull) {
        // We need to cast to the derived type to access its specific state.
        // This is safe because we know the type from the enum.
        PushPullTool* pushPullTool = static_cast<PushPullTool*>(activeTool);
        return pushPullTool->IsPushPullActive(); // Only enable when operation is active
    }
    
    // For other tools (LineTool), keep snapping enabled.
    return true;
}

void ToolManager::OnLeftMouseDown(int mouseX, int mouseY, bool shift, bool ctrl) {
    if (activeTool) activeTool->OnLeftMouseDown(mouseX, mouseY, shift, ctrl);
}

void ToolManager::OnLeftMouseUp(int mouseX, int mouseY, bool shift, bool ctrl) {
    if (activeTool) activeTool->OnLeftMouseUp(mouseX, mouseY, shift, ctrl);
}

void ToolManager::OnRightMouseDown() {
    if (activeTool) activeTool->OnRightMouseDown();
}

void ToolManager::OnMouseMove(int mouseX, int mouseY) {
    if (activeTool) activeTool->OnMouseMove(mouseX, mouseY);
}

void ToolManager::OnKeyDown(SDL_Keycode key, bool shift, bool ctrl) {
    if (activeTool) activeTool->OnKeyDown(key, shift, ctrl);
}

void ToolManager::OnUpdate(const SnapResult& snap) {
    if (activeTool) activeTool->OnUpdate(snap);
}

void ToolManager::RenderUI() {
    if (activeTool) activeTool->RenderUI();
}

void ToolManager::RenderPreview(Renderer& renderer, const SnapResult& snap) {
    if (activeTool) activeTool->RenderPreview(renderer, snap);
}

} // namespace Urbaxio::Tools 