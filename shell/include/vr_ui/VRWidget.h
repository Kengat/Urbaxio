// shell/include/vr_ui/VRWidget.h
#pragma once

#include "vr_ui/IVRWidget.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <functional>

namespace Urbaxio::VRUI {

class VRWidget : public IVRWidget {
public:
    VRWidget(const glm::mat4& localTransform);
    virtual ~VRWidget() = default;

    void SetParentTransform(const glm::mat4& parentTransform) override;
    void SetHovered(bool hovered) override;
    float CheckHit(const VRInputState& input) override = 0;

    bool IsHovered() const { return isHovered_; }
    float GetLastHitDistance() const { return lastHitDistance_; }

protected:
    glm::mat4 localTransform_;
    glm::mat4 parentTransform_;
    glm::mat4 finalTransform_;
    bool isHovered_ = false;
    float lastHitDistance_ = -1.0f;
};

} // namespace Urbaxio::VRUI

