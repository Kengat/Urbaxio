// shell/include/vr_ui/VRPanel.h
#pragma once

#include "vr_ui/IVRWidget.h"
#include <vector>

namespace Urbaxio::VRUI {

class VRPanel : public IVRWidget {
public:
    VRPanel(const std::string& name);

    void Update(const VRInputState& input, const VRUIStyle& style) override;
    void Render(Renderer& renderer, TextRenderer& textRenderer, const glm::mat4& view, const glm::mat4& projection, const VRUIStyle& style) override;
    void SetParentTransform(const glm::mat4& parentTransform) override;
    float CheckHit(const VRInputState& input) override;
    void SetHovered(bool hovered) override;

    void AddWidget(std::unique_ptr<IVRWidget> widget);
    const std::string& GetName() const;
    std::vector<IVRWidget*> GetAllWidgets();

    void SetOffsetTransform(const glm::mat4& offset);
    void StartDrag(const glm::mat4& controllerTransform);
    void UpdateDrag(const glm::mat4& controllerTransform);
    void StopDrag();

    glm::mat4 GetFinalTransform() const { return finalTransform_; }

private:
    std::string name_;
    glm::mat4 offsetTransform_;
    glm::mat4 parentTransform_;
    glm::mat4 finalTransform_;

    std::vector<std::unique_ptr<IVRWidget>> widgets_;
    
    bool isBeingDragged_ = false;
    glm::mat4 dragStartOffsetTransform_;
    glm::mat4 dragStartControllerTransform_;
};

}

