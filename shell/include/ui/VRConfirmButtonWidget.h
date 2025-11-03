#pragma once

#include "ui/IVRWidget.h"
#include <functional>
#include <glm/glm.hpp>

namespace Urbaxio {
    class Renderer;
    class TextRenderer;
}

namespace Urbaxio::UI {

class VRConfirmButtonWidget : public IVRWidget {
public:
    VRConfirmButtonWidget(const glm::vec3& localPos, float diameter, glm::vec3 color, std::function<void()> onClick);
    VRConfirmButtonWidget(const glm::vec3& localPos, float diameter, unsigned int textureId, std::function<void()> onClick, glm::vec3 color = glm::vec3(1.0f));

    void Update(const Ray& localRay, bool isClicked, bool isClickReleased, float stickY) override;
    void Render(Renderer& renderer, TextRenderer& textRenderer, const glm::mat4& panelTransform, const glm::mat4& view, const glm::mat4& projection, float alpha, const std::optional<MaskData>& mask = std::nullopt) override;
    HitResult CheckIntersection(const Ray& localRay) override;
    void HandleClick() override;

    void SetLocalPosition(const glm::vec3& pos) override;
    const glm::vec3& GetLocalPosition() const override;
    glm::vec2 GetSize() const override;

private:
    glm::vec3 localPosition_;
    float diameter_;
    glm::vec3 color_;
    unsigned int textureId_ = 0;
    std::function<void()> onClick_;
    float hoverAlpha_ = 0.0f;
};

} // namespace Urbaxio::UI

