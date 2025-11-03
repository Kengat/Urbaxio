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

class VRToolButtonWidget : public IVRWidget {
public:
    VRToolButtonWidget(const std::string& text, const glm::vec3& localPos, const glm::vec2& size,
                       GLuint textureId,
                       Urbaxio::Tools::ToolType toolType, Urbaxio::Tools::ToolManager& toolManager,
                       std::function<void()> onClick);

    void Update(const Ray& localRay, bool isClicked, bool isClickReleased, float stickY) override;
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

    // Colors for the sphere indicator
    glm::vec3 selectedColor_ = glm::vec3(1.0f, 0.79f, 0.4f);
    glm::vec3 orange_aberration1_ = glm::vec3(1.00f, 0.84f, 0.26f);
    glm::vec3 orange_aberration2_ = glm::vec3(1.0f, 0.1f, 0.1f);

    glm::vec3 inactiveColor_ = glm::vec3(0.3f, 0.75f, 1.0f);
    glm::vec3 blue_aberration1_ = glm::vec3(0.67f, 0.5f, 1.0f);
    glm::vec3 blue_aberration2_ = glm::vec3(0.3f, 1.0f, 0.76f);
};

} // namespace Urbaxio::UI

