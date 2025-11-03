#pragma once

#include <glm/glm.hpp>
#include <string>
#include <functional>
#include <optional>

namespace Urbaxio {
    class Renderer;
    class TextRenderer;
}

namespace Urbaxio::UI {

// Структура для передачи данных маски в виджеты
struct MaskData {
    glm::mat4 transform; // Матрица трансформации области маски
    glm::vec2 size;      // Размер области маски (полная ширина и высота)
    float cornerRadius;
};

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

    virtual void Update(const Ray& localRay, bool isClicked, bool isClickReleased, float stickY) = 0;
    // Unified Render signature with optional mask data
    virtual void Render(Urbaxio::Renderer& renderer, Urbaxio::TextRenderer& textRenderer, const glm::mat4& panelTransform, const glm::mat4& view, const glm::mat4& projection, float alpha, const std::optional<MaskData>& mask = std::nullopt) = 0;
    virtual HitResult CheckIntersection(const Ray& localRay) = 0;
    virtual void HandleClick() = 0;
    
    virtual void SetHover(bool hover) { isHovered_ = hover; }
    virtual bool IsHovered() const { return isHovered_; }

    virtual void SetLocalPosition(const glm::vec3& pos) = 0;
    virtual const glm::vec3& GetLocalPosition() const = 0;
    // Optional: allow layouts to resize widgets; default is no-op
    virtual void SetSize(const glm::vec2& size) {}
    virtual glm::vec2 GetSize() const = 0;

protected:
    bool isHovered_ = false;
};

}

