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

class VRButtonWidget : public IVRWidget {
public:
    VRButtonWidget(const std::string& text, const glm::vec3& localPos, const glm::vec2& size, std::function<void()> onClick);

    void Update(const Ray& localRay, bool isClicked) override;
    void Render(Urbaxio::Renderer& renderer, Urbaxio::TextRenderer& textRenderer, const glm::mat4& panelTransform, const glm::mat4& view, const glm::mat4& projection, float alpha) override;
    HitResult CheckIntersection(const Ray& localRay) override;
    void HandleClick() override;

private:
    std::string text_;
    glm::vec3 localPosition_;
    glm::vec2 size_;
    std::function<void()> onClick_;
};

}

