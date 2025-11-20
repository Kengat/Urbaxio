#pragma once

#include "ui/IVRWidget.h"
#include <string>
#include <functional>
#include <glm/glm.hpp>

namespace Urbaxio {
    class Renderer;
    class TextRenderer;
}

namespace Urbaxio::UI {

class VRToggleWidget : public IVRWidget {
public:
    VRToggleWidget(const std::string& label, const glm::vec3& localPos, const glm::vec2& size, 
                   bool initialValue, std::function<void(bool)> onValueChanged);

    void Update(const Ray& localRay, bool triggerPressed, bool triggerReleased, bool triggerHeld, bool aButtonPressed, float stickY) override;
    void Render(Urbaxio::Renderer& renderer, Urbaxio::TextRenderer& textRenderer, const glm::mat4& panelTransform, const glm::mat4& view, const glm::mat4& projection, float alpha, const std::optional<MaskData>& mask = std::nullopt) const override;
    HitResult CheckIntersection(const Ray& localRay) override;
    void HandleClick() override;

    void SetLocalPosition(const glm::vec3& pos) override;
    const glm::vec3& GetLocalPosition() const override;
    void SetSize(const glm::vec2& size) override;
    glm::vec2 GetSize() const override;

private:
    std::string label_;
    glm::vec3 localPosition_;
    glm::vec2 size_; // Height determines diameter
    
    bool value_;
    std::function<void(bool)> onValueChanged_;
    
    float hoverAlpha_ = 0.0f;
    float animT_ = 0.0f; // 0.0 (Left/False) to 1.0 (Right/True)
};

} // namespace Urbaxio::UI

