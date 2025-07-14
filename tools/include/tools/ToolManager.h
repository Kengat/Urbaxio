#pragma once

#include "tools/ITool.h"
#include <memory>
#include <map>

namespace Urbaxio::Tools {

class ToolManager {
public:
    ToolManager(const ToolContext& context);

    // Activates a tool of a given type.
    void SetTool(ToolType type);
    ITool* GetActiveTool() const;
    ToolType GetActiveToolType() const;
    
    // Check if snapping should be enabled for current tool
    bool ShouldEnableSnapping() const;

    // --- Event Forwarding (called by InputHandler) ---
    void OnLeftMouseDown(int mouseX, int mouseY, bool shift, bool ctrl);
    void OnLeftMouseUp(int mouseX, int mouseY, bool shift, bool ctrl);
    void OnRightMouseDown();
    void OnMouseMove(int mouseX, int mouseY);
    void OnKeyDown(SDL_Keycode key, bool shift, bool ctrl);

    // --- Update and Render (called by main loop) ---
    void OnUpdate(const SnapResult& snap);
    void RenderUI();
    void RenderPreview(Renderer& renderer, const SnapResult& snap);

private:
    std::map<ToolType, std::unique_ptr<ITool>> tools;
    ITool* activeTool = nullptr;
    ToolContext context;
};

} // namespace Urbaxio::Tools 