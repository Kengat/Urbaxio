#pragma once

#include <glad/glad.h> // Include for GLuint definition

#include "ui/IVRWidget.h"

#include "tools/ToolManager.h"

#include <string>
#include <functional>
#include <glm/glm.hpp>

namespace Urbaxio {
    class Renderer;
    class TextRenderer;
}

namespace Urbaxio::UI {

// A struct to hold a complete color theme for a button state
struct ToolButtonColors {
    glm::vec3 base;
    glm::vec3 aberration1;
    glm::vec3 aberration2;
};

class VRToolButtonWidget : public IVRWidget {
public:
    VRToolButtonWidget(const std::string& text, const glm::vec3& localPos, const glm::vec2& size,
                       GLuint textureId,
                       Urbaxio::Tools::ToolType toolType, Urbaxio::Tools::ToolManager& toolManager,
                       std::function<void()> onClick,
                       const ToolButtonColors& selectedColors = { {1.0f, 0.79f, 0.4f}, {1.00f, 0.84f, 0.26f}, {1.0f, 0.1f, 0.1f} },
                       const ToolButtonColors& inactiveColors = { {0.3f, 0.75f, 1.0f}, {0.67f, 0.5f, 1.0f}, {0.3f, 1.0f, 0.76f} });

    void Update(const Ray& localRay, bool triggerPressed, bool triggerReleased, bool triggerHeld, bool aButtonPressed, float stickY) override;
    void Render(Urbaxio::Renderer& renderer, Urbaxio::TextRenderer& textRenderer, const glm::mat4& panelTransform, const glm::mat4& view, const glm::mat4& projection, float alpha, const std::optional<MaskData>& mask = std::nullopt) const override;
    HitResult CheckIntersection(const Ray& localRay) override;
    void HandleClick() override;

    void SetLocalPosition(const glm::vec3& pos) override;
    const glm::vec3& GetLocalPosition() const override;
    void SetSize(const glm::vec2& size) override;
    glm::vec2 GetSize() const override;

private:
    std::string text_;
    glm::vec3 localPosition_;
    glm::vec2 size_;
    Urbaxio::Tools::ToolType toolType_;
    Urbaxio::Tools::ToolManager& toolManager_;
    std::function<void()> onClick_;
    GLuint textureId_ = 0;

    float sphereHoverAlpha_ = 0.0f;
    float textHoverAlpha_ = 0.0f;

    ToolButtonColors selectedColors_;
    ToolButtonColors inactiveColors_;
};

} // namespace Urbaxio::UI

