// shell/src/vr_ui/VRWidget.cpp
#define GLM_ENABLE_EXPERIMENTAL
#include "vr_ui/VRWidget.h"
#include <glm/gtx/intersect.hpp>

namespace Urbaxio::VRUI {

VRWidget::VRWidget(const glm::mat4& localTransform)
    : localTransform_(localTransform), parentTransform_(1.0f), finalTransform_(1.0f) {}

void VRWidget::SetParentTransform(const glm::mat4& parentTransform) {
    parentTransform_ = parentTransform;
    finalTransform_ = parentTransform_ * localTransform_;
}

void VRWidget::SetHovered(bool hovered) {
    isHovered_ = hovered;
}

} // namespace Urbaxio::VRUI

