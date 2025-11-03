#pragma once

#include <glm/glm.hpp>
#include <string>
#include <functional>

namespace Urbaxio {
    class Renderer;
    class TextRenderer;
}

namespace Urbaxio::UI {

struct Ray {
    glm::vec3 origin;
    glm::vec3 direction;
};

struct HitResult {
    bool didHit = false;
    float distance = 0.0f;
    void* hitWidget = nullptr;
};

class IVRWidget {
public:
    virtual ~IVRWidget() = default;

    virtual void Update(const Ray& localRay, bool isClicked) = 0;
    virtual void Render(Urbaxio::Renderer& renderer, Urbaxio::TextRenderer& textRenderer, const glm::mat4& panelTransform, const glm::mat4& view, const glm::mat4& projection, float alpha) = 0;
    virtual HitResult CheckIntersection(const Ray& localRay) = 0;
    virtual void HandleClick() = 0;
    
    virtual void SetHover(bool hover) { isHovered_ = hover; }

    virtual void SetLocalPosition(const glm::vec3& pos) = 0;
    virtual const glm::vec3& GetLocalPosition() const = 0;
    virtual glm::vec2 GetSize() const = 0;

protected:
    bool isHovered_ = false;
};

}

