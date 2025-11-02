// shell/src/vr_ui/VRPanel.cpp
#include "vr_ui/VRPanel.h"
#include "renderer.h"
#include "TextRenderer.h"
#include <glm/gtc/matrix_transform.hpp>

namespace Urbaxio::VRUI {

VRPanel::VRPanel(const std::string& name)
    : name_(name), parentTransform_(1.0f), finalTransform_(1.0f), isBeingDragged_(false) 
{
    offsetTransform_ = glm::mat4(1.0f);
}

void VRPanel::Update(const VRInputState& input, const VRUIStyle& style) {
    finalTransform_ = parentTransform_ * offsetTransform_;
    
    for (auto& widget : widgets_) {
        widget->SetParentTransform(finalTransform_);
    }
}

void VRPanel::Render(Renderer& renderer, TextRenderer& textRenderer, const glm::mat4& view, const glm::mat4& projection, const VRUIStyle& style) {
    glm::mat4 backgroundTransform = finalTransform_ * 
        glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -0.1f, 0.0f)) *
        glm::scale(glm::mat4(1.0f), glm::vec3(0.25f, 0.35f, 1.0f));

    renderer.RenderVRPanel(view, projection, backgroundTransform, style.panelColor, style.panelCornerRadius, style.panelAlpha);

    for (auto& widget : widgets_) {
        widget->Render(renderer, textRenderer, view, projection, style);
    }
}

void VRPanel::SetParentTransform(const glm::mat4& parentTransform) {
    parentTransform_ = parentTransform;
}

void VRPanel::AddWidget(std::unique_ptr<IVRWidget> widget) {
    widgets_.push_back(std::move(widget));
}

const std::string& VRPanel::GetName() const {
    return name_;
}

std::vector<IVRWidget*> VRPanel::GetAllWidgets() {
    std::vector<IVRWidget*> all_widgets;
    for(const auto& w : widgets_) {
        all_widgets.push_back(w.get());
    }
    return all_widgets;
}

void VRPanel::SetOffsetTransform(const glm::mat4& offset) {
    offsetTransform_ = offset;
}

void VRPanel::StartDrag(const glm::mat4& controllerTransform) {
    isBeingDragged_ = true;
    dragStartControllerTransform_ = controllerTransform;
    dragStartOffsetTransform_ = offsetTransform_;
}

void VRPanel::UpdateDrag(const glm::mat4& controllerTransform) {
    if (!isBeingDragged_) return;
    glm::mat4 deltaTransform = controllerTransform * glm::inverse(dragStartControllerTransform_);
    glm::mat4 newFinalTransform = deltaTransform * (parentTransform_ * dragStartOffsetTransform_);
    offsetTransform_ = glm::inverse(parentTransform_) * newFinalTransform;
}

void VRPanel::StopDrag() {
    isBeingDragged_ = false;
}

void VRPanel::SetHovered(bool hovered) {
    // Panel itself doesn't have hover state
}

float VRPanel::CheckHit(const VRInputState& input) {
    return -1.0f;
}

}

