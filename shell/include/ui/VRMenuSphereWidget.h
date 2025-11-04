#pragma once

#include "ui/IVRWidget.h"

#include <functional>
#include <glm/glm.hpp>

namespace Urbaxio {
    class Renderer;
    class TextRenderer;
}

namespace Urbaxio::UI {

class VRMenuSphereWidget : public IVRWidget {
public:

    VRMenuSphereWidget(const glm::vec3& localPos, float diameter, 
                       const glm::vec3& baseColor, const glm::vec3& abColor1, const glm::vec3& abColor2, 
                       std::function<void()> onClick);

    void Update(const Ray& localRay, bool triggerPressed, bool triggerReleased, bool triggerHeld, bool aButtonPressed, float stickY) override;
    void Render(Urbaxio::Renderer& renderer, Urbaxio::TextRenderer& textRenderer, const glm::mat4& panelTransform, const glm::mat4& view, const glm::mat4& projection, float alpha, const std::optional<MaskData>& mask = std::nullopt) const override;
    HitResult CheckIntersection(const Ray& localRay) override;
    void HandleClick() override;

    void SetLocalPosition(const glm::vec3& pos) override;
    const glm::vec3& GetLocalPosition() const override;
    glm::vec2 GetSize() const override;

private:

    glm::vec3 localPosition_;
    float diameter_;
    glm::vec3 baseColor_;
    glm::vec3 aberrationColor1_;
    glm::vec3 aberrationColor2_;
    std::function<void()> onClick_;
    float hoverAlpha_ = 0.0f;
};

} // namespace Urbaxio::UI

