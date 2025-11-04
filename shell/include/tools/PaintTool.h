#pragma once

#include "tools/ITool.h"

namespace Urbaxio::Tools {

class PaintTool : public ITool {
public:
    ToolType GetType() const override { return ToolType::Paint; }
    const char* GetName() const override { return "Paint"; }

    void Activate(const ToolContext& context) override;
    void Deactivate() override;
    void OnLeftMouseDown(int mouseX, int mouseY, bool shift, bool ctrl, const glm::vec3& rayOrigin = {}, const glm::vec3& rayDirection = {}) override;
    void OnUpdate(const SnapResult& snap, const glm::vec3& rayOrigin = {}, const glm::vec3& rayDirection = {}) override;
    void OnMouseMove(int mouseX, int mouseY) override;
    
    // Called from the UI to set the active material
    void SetCurrentMaterial(const std::string& materialName);

private:
    std::string currentMaterialName_ = "Default";
    void updateHover(const glm::vec3& rayOrigin, const glm::vec3& rayDirection);
};

} // namespace Urbaxio::Tools


