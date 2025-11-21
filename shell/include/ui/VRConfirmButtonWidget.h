#pragma once

#include "ui/IVRWidget.h"
#include <functional>
#include <string>
#include <glm/glm.hpp>

namespace Urbaxio {
    class Renderer;
    class TextRenderer;
}

namespace Urbaxio::UI {

// ДОБАВЬ ЭТОТ ENUM

enum class DepthEffect {

    CONVEX,  // "Выпуклый", парит перед сферой

    CONCAVE  // "Вогнутый", эффект глубины

};

class VRConfirmButtonWidget : public IVRWidget {
public:
    VRConfirmButtonWidget(const glm::vec3& localPos, float diameter, glm::vec3 color, std::function<void()> onClick);
    VRConfirmButtonWidget(const glm::vec3& localPos, float diameter, unsigned int textureId, std::function<void()> onClick, glm::vec3 color = glm::vec3(1.0f));

    void Update(const Ray& localRay, bool triggerPressed, bool triggerReleased, bool triggerHeld, bool aButtonPressed, float stickY) override;
    void Render(Renderer& renderer, TextRenderer& textRenderer, const glm::mat4& panelTransform, const glm::mat4& view, const glm::mat4& projection, float alpha, const std::optional<MaskData>& mask = std::nullopt) const override;
    HitResult CheckIntersection(const Ray& localRay) override;
    void HandleClick() override;

    void SetLocalPosition(const glm::vec3& pos) override;
    const glm::vec3& GetLocalPosition() const override;
    void SetSize(const glm::vec2& size) override;
    glm::vec2 GetSize() const override;

    void SetIconOffsetUseCameraForward(bool useCameraForward) { iconOffsetUseCameraForward_ = useCameraForward; }

    // ДОБАВЬ ЭТОТ МЕТОД

    void setDepthEffect(DepthEffect effect);

    void SetFadesWhenNotHovered(bool fades);

    void SetColor(const glm::vec3& newColor);

    void SetDepthStrength(float forwardFactorScale, float disparityScale);

    void SetLabel(const std::string& text, bool alwaysVisible, float verticalOffset = 0.0f, float horizontalOffset = 0.0f);

private:
    glm::vec3 localPosition_;
    float diameter_;
    glm::vec3 color_;
    unsigned int textureId_ = 0;
    std::function<void()> onClick_;
    float hoverAlpha_ = 0.0f;
    bool iconOffsetUseCameraForward_ = false;
    // ДОБАВЬ ЭТОТ ЧЛЕН КЛАССА

    DepthEffect effect_ = DepthEffect::CONVEX; // По умолчанию эффект "выпуклый"

    bool fadesWhenNotHovered_ = false;

    float forwardFactorScale_ = 1.0f;
    float disparityScale_ = 1.0f;

    std::string labelText_;
    bool labelAlwaysVisible_ = false;
    float labelVerticalOffset_ = 0.0f;
    float labelHorizontalOffset_ = 0.0f;

};

} // namespace Urbaxio::UI

