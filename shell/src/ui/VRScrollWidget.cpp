#define GLM_ENABLE_EXPERIMENTAL
#include "ui/VRScrollWidget.h"
#include "renderer.h"
#include "TextRenderer.h"
#include "snapping.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glad/glad.h>
#include <glm/gtx/intersect.hpp>
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

void VRScrollWidget::RecalculateContentLayout() {
    totalContentHeight_ = 0.0f;
    const float spacing = 0.005f;

    if (!children_.empty()) {
        for (const auto& child : children_) {
            totalContentHeight_ += child->GetSize().y;
        }
        totalContentHeight_ += (children_.size() - 1) * spacing;
    }
    
    float currentY = size_.y / 2.0f;
    for (auto& child : children_) {
        const auto& childSize = child->GetSize();
        child->SetLocalPosition({0.0f, currentY - childSize.y / 2.0f, 0.001f});
        currentY -= (childSize.y + spacing);
    }

    float maxScroll = std::max(0.0f, totalContentHeight_ - size_.y);
    scrollOffset_ = std::clamp(scrollOffset_, 0.0f, maxScroll);
}

void VRScrollWidget::Update(const Ray& localRay, bool isClicked, bool isClickReleased, float stickY) {
	HitResult hit = CheckIntersection(localRay);
	bool isInside = hit.didHit;
	float maxScroll = std::max(0.0f, totalContentHeight_ - size_.y);

	const float STICK_SCROLL_SPEED = 0.05f;
	const float STICK_DEAD_ZONE = 0.2f;
	if (isInside && std::abs(stickY) > STICK_DEAD_ZONE) {
		scrollOffset_ += stickY * STICK_SCROLL_SPEED;
		scrollOffset_ = std::clamp(scrollOffset_, 0.0f, maxScroll);
	}

	const float kDragThreshold = 0.003f;
	if (isClicked && isInside) {
		isDraggingScroll_ = true;
		dragMoved_ = false;
		dragCapturedChild_ = reinterpret_cast<IVRWidget*>(hit.hitWidget);
		dragStartPoint_ = localRay.origin + localRay.direction * hit.distance;
		scrollOffsetAtDragStart_ = scrollOffset_;
	}
	
	if (isClickReleased) {
		if (isDraggingScroll_ && !dragMoved_ && dragCapturedChild_) {
			dragCapturedChild_->HandleClick();
		}
		isDraggingScroll_ = false;
		dragCapturedChild_ = nullptr;
	}

	if (isDraggingScroll_) {
		float t_plane;
		if (glm::intersectRayPlane(localRay.origin, localRay.direction, localPosition_, glm::vec3(0, 0, 1), t_plane)) {
			glm::vec3 currentDragPoint = localRay.origin + localRay.direction * t_plane;
			float deltaY = currentDragPoint.y - dragStartPoint_.y;

			if (std::abs(deltaY) > kDragThreshold) dragMoved_ = true;

			scrollOffset_ = std::clamp(scrollOffsetAtDragStart_ + deltaY, 0.0f, maxScroll);
		}
	}
	
	IVRWidget* newHovered = (!isDraggingScroll_ && isInside) ? reinterpret_cast<IVRWidget*>(hit.hitWidget) : nullptr;
	if (hoveredChild_ != newHovered) {
		if (hoveredChild_) hoveredChild_->SetHover(false);
		hoveredChild_ = newHovered;
		if (hoveredChild_) hoveredChild_->SetHover(true);
	}
	
	for (auto& child : children_) {
		child->Update(localRay, false, false, 0.0f);
	}
}

void VRScrollWidget::Render(Renderer& renderer, TextRenderer& textRenderer, const glm::mat4& panelTransform, const glm::mat4& view, const glm::mat4& projection, float alpha, const std::optional<MaskData>& scissor) {
    glm::mat4 maskTransform = panelTransform * glm::translate(glm::mat4(1.0f), localPosition_);

    MaskData maskData = {
        maskTransform,
        size_,
        0.0f
    };

    glm::mat4 contentTransform = panelTransform * glm::translate(glm::mat4(1.0f), localPosition_ + glm::vec3(0.0f, scrollOffset_, 0.0f));

    for (auto& child : children_) {
        child->Render(renderer, textRenderer, contentTransform, view, projection, alpha, maskData);
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
            result.hitWidget = nullptr;

            Ray contentRay = localRay;
            contentRay.origin -= localPosition_;
            contentRay.origin.y -= scrollOffset_;

            float closestChildHit = std::numeric_limits<float>::max();
            for (auto& child : children_) {
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
        hoveredChild_->HandleClick();
    }
}

void VRScrollWidget::SetLocalPosition(const glm::vec3& pos) { localPosition_ = pos; }
const glm::vec3& VRScrollWidget::GetLocalPosition() const { return localPosition_; }
void VRScrollWidget::SetSize(const glm::vec2& size) { size_ = size; }
glm::vec2 VRScrollWidget::GetSize() const { return size_; }

} // namespace Urbaxio::UI

