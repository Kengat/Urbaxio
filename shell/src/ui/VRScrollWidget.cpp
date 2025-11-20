#define GLM_ENABLE_EXPERIMENTAL
#include "ui/VRScrollWidget.h"
#include "renderer.h"
#include "TextRenderer.h"
#include "snapping.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glad/glad.h>
#include <glm/gtx/intersect.hpp>
#include <glm/gtx/norm.hpp>
#include <algorithm>
#include <numeric>
#include <limits>
#include <iostream>

namespace Urbaxio::UI {

VRScrollWidget::VRScrollWidget(const glm::vec3& localPos, const glm::vec2& size)
    : localPosition_(localPos), size_(size) {}

void VRScrollWidget::AddWidget(std::unique_ptr<IVRWidget> widget) {
    children_.push_back(std::move(widget));
    RecalculateContentLayout();
}
void VRScrollWidget::ClearChildren() {
    children_.clear();
}

void VRScrollWidget::ClearState() {
    hoveredChild_ = nullptr;
    clickedWidget_ = nullptr;
    isAwaitingDrag_ = false;
    isDraggingScroll_ = false;
    scrollOffset_ = 0.0f;
    totalContentHeight_ = 0.0f;
    totalContentWidth_ = 0.0f;
}

ScrollDirection VRScrollWidget::DetermineScrollDirection() const {
    // If width > height, panel is horizontal (landscape), use horizontal scroll
    // If height >= width, panel is vertical (portrait), use vertical scroll
    return (size_.x > size_.y) ? ScrollDirection::HORIZONTAL : ScrollDirection::VERTICAL;
}

void VRScrollWidget::RecalculateContentLayout() {
    ScrollDirection direction = DetermineScrollDirection();
    
    if (layout_) {
        // Use layout to position children
        layout_->Apply(children_, size_);
        
        // Calculate total content size from positioned widgets
        if (children_.empty()) {
            totalContentHeight_ = 0.0f;
            totalContentWidth_ = 0.0f;
        } else {
            float minX = std::numeric_limits<float>::max();
            float maxX = std::numeric_limits<float>::lowest();
            float minY = std::numeric_limits<float>::max();
            float maxY = std::numeric_limits<float>::lowest();
            
            for (const auto& child : children_) {
                const auto& pos = child->GetLocalPosition();
                const auto& childSize = child->GetSize();
                minX = std::min(minX, pos.x - childSize.x / 2.0f);
                maxX = std::max(maxX, pos.x + childSize.x / 2.0f);
                minY = std::min(minY, pos.y - childSize.y / 2.0f);
                maxY = std::max(maxY, pos.y + childSize.y / 2.0f);
            }
            totalContentWidth_ = maxX - minX;
            totalContentHeight_ = maxY - minY;
        }
    } else {
        // Old vertical layout logic (kept for backward compatibility)
        totalContentHeight_ = 0.0f;
        totalContentWidth_ = 0.0f;
        const float spacing = 0.005f;

        if (!children_.empty()) {
            if (direction == ScrollDirection::VERTICAL) {
            for (const auto& child : children_) {
                totalContentHeight_ += child->GetSize().y;
            }
            totalContentHeight_ += (children_.size() - 1) * spacing;
        
        float currentY = totalContentHeight_ / 2.0f;
        for (auto& child : children_) {
            const auto& childSize = child->GetSize();
            currentY -= childSize.y / 2.0f;
            child->SetLocalPosition({0.0f, currentY, 0.001f});
            currentY -= (childSize.y / 2.0f + spacing);
        }
            } else {
                // Horizontal layout
                for (const auto& child : children_) {
                    totalContentWidth_ += child->GetSize().x;
                }
                totalContentWidth_ += (children_.size() - 1) * spacing;
                
                float currentX = -totalContentWidth_ / 2.0f;
                for (auto& child : children_) {
                    const auto& childSize = child->GetSize();
                    currentX += childSize.x / 2.0f;
                    child->SetLocalPosition({currentX, 0.0f, 0.001f});
                    currentX += (childSize.x / 2.0f + spacing);
                }
            }
        }
    }

    ScrollDirection dir = DetermineScrollDirection();
    float maxScroll = (dir == ScrollDirection::VERTICAL) 
        ? std::max(0.0f, totalContentHeight_ - size_.y)
        : std::max(0.0f, totalContentWidth_ - size_.x);
    scrollOffset_ = std::clamp(scrollOffset_, 0.0f, maxScroll);
}

void VRScrollWidget::Update(const Ray& localRay, bool triggerPressed, bool triggerReleased, bool triggerHeld, bool aButtonPressed, float stickY) {
    ScrollDirection direction = DetermineScrollDirection();
    
    Ray contentRay = localRay;
    contentRay.origin -= localPosition_;
    
    // FIX: Apply same offset logic as in Render
    if (direction == ScrollDirection::VERTICAL) {
        float baseOffset = (size_.y / 2.0f) - (totalContentHeight_ / 2.0f);
        contentRay.origin.y -= (baseOffset + scrollOffset_);
    } else {
        float baseOffset = -(size_.x / 2.0f) + (totalContentWidth_ / 2.0f);
        contentRay.origin.x -= (baseOffset - scrollOffset_);
    }

    HitResult hit = CheckIntersection(localRay);
    bool isInside = hit.didHit;
    
    float maxScroll = (direction == ScrollDirection::VERTICAL)
        ? std::max(0.0f, totalContentHeight_ - size_.y)
        : std::max(0.0f, totalContentWidth_ - size_.x);

    // Stick scrolling (only when NOT holding trigger)
    const float STICK_SCROLL_SPEED = 0.05f;
    const float STICK_DEAD_ZONE = 0.2f;
    if (isInside && !triggerHeld && std::abs(stickY) > STICK_DEAD_ZONE) {
        scrollOffset_ += stickY * STICK_SCROLL_SPEED;
        scrollOffset_ = std::clamp(scrollOffset_, 0.0f, maxScroll);
    }

    const float kDragThresholdSq = 0.003f * 0.003f;

    // On trigger press: remember the widget we're hovering AND start awaiting drag
    if (triggerPressed && isInside) {
        isAwaitingDrag_ = true;
        dragStartPoint_ = localRay.origin + localRay.direction * hit.distance;
        scrollOffsetAtDragStart_ = scrollOffset_;
        // NEW: Remember which widget was clicked
        clickedWidget_ = reinterpret_cast<IVRWidget*>(hit.hitWidget);
    }

    // While holding trigger: check if we exceed threshold to start scrolling
    if (triggerHeld && isAwaitingDrag_) {
        float t_plane;
        if (glm::intersectRayPlane(localRay.origin, localRay.direction, localPosition_, glm::vec3(0, 0, 1), t_plane)) {
            glm::vec3 currentDragPoint = localRay.origin + localRay.direction * t_plane;
            if (glm::distance2(currentDragPoint, dragStartPoint_) > kDragThresholdSq) {
                // We've moved enough - start actual scrolling
                isDraggingScroll_ = true;
                isAwaitingDrag_ = false;
                clickedWidget_ = nullptr; // Cancel click
            }
        }
    }

    // Active scrolling
    if (triggerHeld && isDraggingScroll_) {
        float t_plane;
        if (glm::intersectRayPlane(localRay.origin, localRay.direction, localPosition_, glm::vec3(0, 0, 1), t_plane)) {
            glm::vec3 currentDragPoint = localRay.origin + localRay.direction * t_plane;
            
            if (direction == ScrollDirection::VERTICAL) {
            float deltaY = currentDragPoint.y - dragStartPoint_.y;
            scrollOffset_ = std::clamp(scrollOffsetAtDragStart_ + deltaY, 0.0f, maxScroll);
            } else {
                float deltaX = currentDragPoint.x - dragStartPoint_.x;
                scrollOffset_ = std::clamp(scrollOffsetAtDragStart_ - deltaX, 0.0f, maxScroll); // Inverted
            }
        }
    }

    // On trigger release: if we didn't scroll, it's a click!
    if (triggerReleased) {
        if (isAwaitingDrag_ && clickedWidget_) {
            clickedWidget_->HandleClick();
        }
        
        isAwaitingDrag_ = false;
        isDraggingScroll_ = false;
        clickedWidget_ = nullptr;
    }

    // Hover logic
    IVRWidget* newHovered = nullptr;
    if (isDraggingScroll_) {
        newHovered = nullptr;
    } else if (isAwaitingDrag_ && clickedWidget_) {
        newHovered = clickedWidget_;
    } else if (isInside) {
        newHovered = reinterpret_cast<IVRWidget*>(hit.hitWidget);
    }
    
    if (hoveredChild_ != newHovered) {
        if (hoveredChild_) hoveredChild_->SetHover(false);
        hoveredChild_ = newHovered;
        if (hoveredChild_) hoveredChild_->SetHover(true);
    }

    // Update children
    for (auto& child : children_) {
        bool childIsHovered = (hoveredChild_ == child.get());
        child->Update(contentRay, triggerPressed && childIsHovered, triggerReleased, triggerHeld, aButtonPressed && childIsHovered, stickY);
    }
}

void VRScrollWidget::Render(Renderer& renderer, TextRenderer& textRenderer, const glm::mat4& panelTransform, const glm::mat4& view, const glm::mat4& projection, float alpha, const std::optional<MaskData>& scissor) const {
    ScrollDirection direction = DetermineScrollDirection();
    
    // --- START OF MODIFICATION: Adaptive mask padding based on orientation ---
    const float LARGE_PADDING = 0.02f;  // Wider sides (perpendicular to scroll)
    const float SMALL_PADDING = 0.005f; // Narrower sides (parallel to scroll)
    
    glm::vec2 expandedSize;
    if (direction == ScrollDirection::VERTICAL) {
        // Vertical scroll: wider on left/right, narrower on top/bottom
        expandedSize = size_ + glm::vec2(LARGE_PADDING * 2.0f, SMALL_PADDING * 2.0f);
    } else {
        // Horizontal scroll: narrower on left/right, wider on top/bottom
        expandedSize = size_ + glm::vec2(SMALL_PADDING * 2.0f, LARGE_PADDING * 2.0f);
    }
    // --- END OF MODIFICATION ---

    glm::mat4 maskTransform = panelTransform * glm::translate(glm::mat4(1.0f), localPosition_);

    MaskData maskData = {
        maskTransform,
        expandedSize,
        0.0f
    };

    // FIX: Calculate base offset to align content start with panel start
    glm::vec3 scrollVec;
    if (direction == ScrollDirection::VERTICAL) {
        // Align top of content with top of panel
        float baseOffset = (size_.y / 2.0f) - (totalContentHeight_ / 2.0f);
        scrollVec = glm::vec3(0.0f, baseOffset + scrollOffset_, 0.0f);
    } else {
        // Align left of content with left of panel
        float baseOffset = -(size_.x / 2.0f) + (totalContentWidth_ / 2.0f);
        scrollVec = glm::vec3(baseOffset - scrollOffset_, 0.0f, 0.0f);
    }
    
    glm::mat4 contentTransform = panelTransform * glm::translate(glm::mat4(1.0f), localPosition_ + scrollVec);

    // Sort children by distance from camera
    glm::vec3 cameraPos = glm::vec3(glm::inverse(view)[3]);
    
    struct ChildWithDepth {
        IVRWidget* child;
        float depth;
    };
    
    std::vector<ChildWithDepth> sortedChildren;
    for (auto& child : children_) {
        // --- NEW: Skip invisible children during render ---
        if (!child->IsVisible()) continue;
        // --------------------------------------------------

        glm::vec3 childWorldPos = contentTransform * glm::vec4(child->GetLocalPosition(), 1.0f);
        float distance = glm::distance(cameraPos, childWorldPos);
        sortedChildren.push_back({child.get(), distance});
    }
    
    std::sort(sortedChildren.begin(), sortedChildren.end(), 
              [](const ChildWithDepth& a, const ChildWithDepth& b) { return a.depth > b.depth; });

    for (const auto& item : sortedChildren) {
        item.child->Render(renderer, textRenderer, contentTransform, view, projection, alpha, maskData);
    }
}

HitResult VRScrollWidget::CheckIntersection(const Ray& localRay) {
    HitResult result;
    float t_plane;
    if (glm::intersectRayPlane(localRay.origin, localRay.direction, localPosition_, glm::vec3(0, 0, 1), t_plane) && t_plane > 0) {
        glm::vec3 hitPointOnPanel = localRay.origin + localRay.direction * t_plane;
        if (std::abs(hitPointOnPanel.x - localPosition_.x) <= size_.x * 0.5f && std::abs(hitPointOnPanel.y - localPosition_.y) <= size_.y * 0.5f) {
            result.didHit = true;
            result.distance = t_plane;
            result.hitWidget = this;

            ScrollDirection direction = DetermineScrollDirection();
            Ray contentRay = localRay;
            contentRay.origin -= localPosition_;
            
            // FIX: Apply same offset logic
            if (direction == ScrollDirection::VERTICAL) {
                float baseOffset = (size_.y / 2.0f) - (totalContentHeight_ / 2.0f);
                contentRay.origin.y -= (baseOffset + scrollOffset_);
            } else {
                float baseOffset = -(size_.x / 2.0f) + (totalContentWidth_ / 2.0f);
                contentRay.origin.x -= (baseOffset - scrollOffset_);
            }

            float closestChildHit = std::numeric_limits<float>::max();
            for (auto& child : children_) {
                // --- NEW: Skip invisible children during hit test ---
                if (!child->IsVisible()) continue;
                // ----------------------------------------------------

                HitResult childHit = child->CheckIntersection(contentRay);
                if (childHit.didHit && childHit.distance < closestChildHit) {
                    closestChildHit = childHit.distance;
                    result.hitWidget = childHit.hitWidget;
                }
            }
        }
    }
    return result;
}

void VRScrollWidget::HandleClick() {
    if (hoveredChild_) {
        bool childIsAlive = false;
        for (const auto& child : children_) {
            if (child.get() == hoveredChild_) {
                childIsAlive = true;
                break;
            }
        }
        if (childIsAlive) {
            hoveredChild_->HandleClick();
        } else {
            hoveredChild_ = nullptr;
        }
    }
}

void VRScrollWidget::SetLocalPosition(const glm::vec3& pos) { localPosition_ = pos; }
const glm::vec3& VRScrollWidget::GetLocalPosition() const { return localPosition_; }
void VRScrollWidget::SetSize(const glm::vec2& size) { 
    size_ = size; 
    RecalculateContentLayout();
}
glm::vec2 VRScrollWidget::GetSize() const { return size_; }

void VRScrollWidget::SetLayout(std::unique_ptr<ILayout> layout) {
    layout_ = std::move(layout);
}

} // namespace Urbaxio::UI
