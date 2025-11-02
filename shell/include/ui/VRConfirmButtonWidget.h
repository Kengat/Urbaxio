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

    void Update(const Ray& localRay, bool isClicked) override;
    void Render(Renderer& renderer, TextRenderer& textRenderer, const glm::mat4& panelTransform, const glm::mat4& view, const glm::mat4& projection, float alpha) override;
    HitResult CheckIntersection(const Ray& localRay) override;
    void HandleClick() override;

private:
    glm::vec3 localPosition_;
    float diameter_;
    glm::vec3 color_;
    std::function<void()> onClick_;
};

} // namespace Urbaxio::UI

