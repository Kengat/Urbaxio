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

class VRSliderWidget : public IVRWidget {
public:
    // step == 0.0f means continuous movement
    VRSliderWidget(const std::string& label, const glm::vec3& localPos, const glm::vec2& size, 
                   float minVal, float maxVal, float step, float initialVal, 
                   std::function<void(float)> onValueChanged);

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
    glm::vec2 size_;
    
    float minVal_;
    float maxVal_;
    float step_; // 0 for smooth
    float value_;
    
    std::function<void(float)> onValueChanged_;
    
    bool isDragging_ = false;
    float hoverAlpha_ = 0.0f;
};

} // namespace Urbaxio::UI

