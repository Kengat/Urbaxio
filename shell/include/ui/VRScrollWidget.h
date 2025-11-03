#pragma once

#include "ui/IVRWidget.h"
#include <vector>
#include <memory>

namespace Urbaxio::UI {

class VRScrollWidget : public IVRWidget {
public:
    VRScrollWidget(const glm::vec3& localPos, const glm::vec2& size);

    void AddWidget(std::unique_ptr<IVRWidget> widget);
    void RecalculateContentLayout();

    void Update(const Ray& localRay, bool isClicked, bool isClickReleased, float stickY) override;
    void Render(Renderer& renderer, TextRenderer& textRenderer, const glm::mat4& panelTransform, const glm::mat4& view, const glm::mat4& projection, float alpha, const std::optional<MaskData>& scissor = std::nullopt) const override;
    
    HitResult CheckIntersection(const Ray& localRay) override;
    void HandleClick() override;

    void SetLocalPosition(const glm::vec3& pos) override;
    const glm::vec3& GetLocalPosition() const override;
    void SetSize(const glm::vec2& size) override;
    glm::vec2 GetSize() const override;

private:
    glm::vec3 localPosition_;
    glm::vec2 size_;
    std::vector<std::unique_ptr<IVRWidget>> children_;
    IVRWidget* hoveredChild_ = nullptr;

    // Scrolling state
    float scrollOffset_ = 0.0f;
    float totalContentHeight_ = 0.0f;
    bool isDraggingScroll_ = false;
    glm::vec3 dragStartPoint_;
    float scrollOffsetAtDragStart_ = 0.0f;
    
    // Added fields for drag capture and movement threshold
    IVRWidget* dragCapturedChild_ = nullptr;
    bool dragMoved_ = false;
    int lastStickDirection_ = 0;
};

} // namespace Urbaxio::UI
