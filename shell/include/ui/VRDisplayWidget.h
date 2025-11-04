#pragma once

#include "ui/IVRWidget.h"
#include <string>
#include <glm/glm.hpp>

namespace Urbaxio {
    class Renderer;
    class TextRenderer;
}

namespace Urbaxio::UI {

class VRDisplayWidget : public IVRWidget {
public:
    VRDisplayWidget(const glm::vec3& localPos, const glm::vec2& size, std::string& textValue);

    void Update(const Ray& localRay, bool triggerPressed, bool triggerReleased, bool triggerHeld, bool aButtonPressed, float stickY) override;
    void Render(Renderer& renderer, TextRenderer& textRenderer, const glm::mat4& panelTransform, const glm::mat4& view, const glm::mat4& projection, float alpha, const std::optional<MaskData>& mask = std::nullopt) const override;
    HitResult CheckIntersection(const Ray& localRay) override;
    void HandleClick() override;

    void SetLocalPosition(const glm::vec3& pos) override;
    const glm::vec3& GetLocalPosition() const override;
    glm::vec2 GetSize() const override;

private:
    glm::vec3 localPosition_;
    glm::vec2 size_;
    std::string& textValue_;
};

} // namespace Urbaxio::UI

