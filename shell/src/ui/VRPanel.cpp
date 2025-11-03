#define GLM_ENABLE_EXPERIMENTAL
#include "ui/VRPanel.h"
#include "renderer.h"
#include "TextRenderer.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/intersect.hpp>
#include <limits>
#include <cmath>
#include <iostream>
#include <iomanip>

namespace Urbaxio::UI {

VRPanel::VRPanel(const std::string& name, const std::string& displayName, const glm::vec2& size, const glm::mat4& offsetTransform, float cornerRadius, unsigned int grabIcon, unsigned int closeIcon, unsigned int minimizeIcon)
    : name_(name), displayName_(displayName), size_(size), offsetTransform_(offsetTransform), cornerRadius_(cornerRadius), hoveredWidget_(nullptr), isGrabbing(false) {
    
    const float handleDiameter = 0.02f;
    const float handleSpacing = 0.025f;

    glm::vec3 closePos(size_.x * 0.5f - handleDiameter * 0.5f, size_.y * 0.5f - handleDiameter * 0.5f, 0.01f);
    glm::vec3 minimizePos = closePos - glm::vec3(handleSpacing, 0, 0);
    glm::vec3 grabPos = minimizePos - glm::vec3(handleSpacing, 0, 0);
    glm::vec3 resizePos(size_.x * 0.5f - handleDiameter * 0.5f, -size_.y * 0.5f + handleDiameter * 0.5f, 0.01f);
    
    closeHandle_ = std::make_unique<VRConfirmButtonWidget>(closePos, handleDiameter, closeIcon, nullptr, glm::vec3(1.00f, 0.20f, 0.32f));
    minimizeHandle_ = std::make_unique<VRConfirmButtonWidget>(minimizePos, handleDiameter, minimizeIcon, nullptr);
    grabHandle_ = std::make_unique<VRConfirmButtonWidget>(grabPos, handleDiameter, grabIcon, nullptr);
    resizeHandle_ = std::make_unique<VRConfirmButtonWidget>(resizePos, handleDiameter, glm::vec3(1.0f), nullptr);
}

void VRPanel::AddWidget(std::unique_ptr<IVRWidget> widget) {
    widgets_.push_back(std::move(widget));
}

void VRPanel::SetLayout(std::unique_ptr<ILayout> layout) {
    layout_ = std::move(layout);
}

void VRPanel::RecalculateLayout() {
    if (layout_) {
        layout_->Apply(widgets_, size_);
    }
}

void VRPanel::Update(const Ray& worldRay, const glm::mat4& parentTransform, const glm::mat4& interactionTransform, bool isClicked, bool isClickReleased, bool aButtonIsPressed, bool bButtonIsPressed) {
    // --- Animation Logic ---
    const float ANIM_SPEED = 0.1f;
    float targetT = minimizeTargetState_ ? 1.0f : 0.0f;
    minimizeT_ += (targetT - minimizeT_) * ANIM_SPEED;

    if (isGrabbing) {
        glm::mat4 deltaTransform = interactionTransform * glm::inverse(grabbedControllerInitialTransform);
        transform = deltaTransform * grabbedInitialTransform;
        offsetTransform_ = glm::inverse(parentTransform) * transform;
    } else {
        transform = parentTransform * offsetTransform_;
    }

    // --- Resize/Proportion Drag Logic ---
    if (isResizing_ || isChangingProportions_) {
        // Project the current controller position onto the panel's plane
        glm::vec3 panelOrigin = panelCenterAtResizeStart_;
        glm::vec3 panelNormal = -glm::normalize(glm::vec3(transform[2]));
        
        float t;
        glm::intersectRayPlane(worldRay.origin, worldRay.direction, panelOrigin, panelNormal, t);
        glm::vec3 controllerPosOnPlane = worldRay.origin + worldRay.direction * t;

        // Decompose the original offset transform to apply new scale
        glm::vec3 scale, translation, skew;
        glm::quat orientation;
        glm::vec4 perspective;
        (void)glm::decompose(offsetTransform_, scale, orientation, translation, skew, perspective);

        if (isResizing_) {
            // --- UNIFORM SCALE (proportional) ---
            float currentDist = glm::distance(panelCenterAtResizeStart_, controllerPosOnPlane);
            float initialDist = glm::distance(panelCenterAtResizeStart_, resizeStartControllerPos_);
            float scaleFactor = (initialDist > 0.01f) ? (currentDist / initialDist) : 1.0f;
            scaleFactor = glm::max(0.1f, scaleFactor);
            glm::vec3 newScale = resizeStartScale_ * scaleFactor;
            
            offsetTransform_ = glm::translate(glm::mat4(1.0f), translation) * glm::mat4_cast(orientation) * glm::scale(glm::mat4(1.0f), newScale);
        
        } else { // isChangingProportions_ (Non-proportional, affects size_)
            // --- ADDITIVE DELTA WITH SCALE COMPENSATION ---
            // Vectors from panel center to current and start controller positions (on panel plane)
            glm::vec3 currentVec = controllerPosOnPlane - panelCenterAtResizeStart_;
            glm::vec3 initialVec = resizeStartControllerPos_ - panelCenterAtResizeStart_;
            
            // Absolute distances along panel axes captured at drag start (world units)
            float currentDistX = std::abs(glm::dot(currentVec, initialPanelXDir_));
            float currentDistY = std::abs(glm::dot(currentVec, initialPanelYDir_));
            float initialDistX = std::abs(glm::dot(initialVec, initialPanelXDir_));
            float initialDistY = std::abs(glm::dot(initialVec, initialPanelYDir_));

            // Change in half-extent along each axis (world units)
            float deltaX_world = currentDistX - initialDistX;
            float deltaY_world = currentDistY - initialDistY;

            // Compensate for starting panel local scale so deltas correctly map to local size units
            float scaleCompensatorX = (resizeStartScale_.x > 1e-5f) ? (1.0f / resizeStartScale_.x) : 1.0f;
            float scaleCompensatorY = (resizeStartScale_.y > 1e-5f) ? (1.0f / resizeStartScale_.y) : 1.0f;

            // New size is start size plus twice the compensated delta (panel grows symmetrically)
            const float minSize = 0.05f;
            float newWidth  = resizeStartSize_.x + (deltaX_world * 2.0f) * scaleCompensatorX;
            float newHeight = resizeStartSize_.y + (deltaY_world * 2.0f) * scaleCompensatorY;
            size_.x = glm::max(minSize, newWidth);
            size_.y = glm::max(minSize, newHeight);

            RecalculateLayout();
        }
    }

    if (isClickReleased) {
        isResizing_ = false;
        isChangingProportions_ = false;
        isGrabbing = false;
    }

    glm::mat4 invTransform = glm::inverse(transform);
    Ray localRay;
    localRay.origin = invTransform * glm::vec4(worldRay.origin, 1.0f);
    localRay.direction = glm::normalize(glm::vec3(invTransform * glm::vec4(worldRay.direction, 0.0f)));

    bool clickConsumed = false;

    // Always update all handles
    HitResult grabHit = grabHandle_->CheckIntersection(localRay);
    HitResult resizeHit = resizeHandle_->CheckIntersection(localRay);
    HitResult minimizeHit = minimizeHandle_->CheckIntersection(localRay);
    HitResult closeHit = closeHandle_->CheckIntersection(localRay);
    
    grabHandle_->SetHover(grabHit.didHit);
    resizeHandle_->SetHover(resizeHit.didHit);
    minimizeHandle_->SetHover(minimizeHit.didHit);
    closeHandle_->SetHover(closeHit.didHit);
    
    grabHandle_->Update(localRay, false);
    resizeHandle_->Update(localRay, false);
    minimizeHandle_->Update(localRay, false);
    closeHandle_->Update(localRay, false);

    if (isClicked) {
        if (grabHit.didHit) {
            isGrabbing = true;
            grabbedInitialTransform = transform;
            grabbedControllerInitialTransform = interactionTransform;
            clickConsumed = true;
        } else if (resizeHit.didHit) {
            clickConsumed = true;
            
            if (bButtonIsPressed) {
                // Log panel info to console
                std::cout << "\n--- PANEL INFO: " << displayName_ << " (" << name_ << ") ---\n";
                std::cout << std::fixed << std::setprecision(3);

                glm::vec3 scale, translation, skew;
                glm::quat orientation;
                glm::vec4 perspective;
                (void)glm::decompose(offsetTransform_, scale, orientation, translation, skew, perspective);
                
                glm::vec3 eulerAnglesRad = glm::eulerAngles(orientation);
                glm::vec3 eulerAnglesDeg = glm::degrees(eulerAnglesRad);

                std::cout << "Offset Transform:\n";
                std::cout << "  Translation: glm::vec3(" << translation.x << "f, " << translation.y << "f, " << translation.z << "f)\n";
                std::cout << "  Scale:       glm::vec3(" << scale.x << "f, " << scale.y << "f, " << scale.z << "f)\n";
                std::cout << "  Rotation (Euler Deg): glm::radians(glm::vec3(" << eulerAnglesDeg.x << "f, " << eulerAnglesDeg.y << "f, " << eulerAnglesDeg.z << "f))\n";

                std::cout << "\nConstructor Parameters:\n";
                std::cout << "  Size:          glm::vec2(" << size_.x << "f, " << size_.y << "f)\n";
                std::cout << "  Corner Radius: " << cornerRadius_ << "f\n";

                std::cout << "-------------------------------------------\n\n";
            } else {
        // --- START RESIZE OR PROPORTION CHANGE ---
        panelCenterAtResizeStart_ = glm::vec3(transform[3]);
        
        // Project start point onto panel plane
        glm::vec3 panelNormal = -glm::normalize(glm::vec3(transform[2]));
        float t;
        glm::intersectRayPlane(worldRay.origin, worldRay.direction, panelCenterAtResizeStart_, panelNormal, t);
        resizeStartControllerPos_ = worldRay.origin + worldRay.direction * t;

        // Store initial scale
        (void)glm::decompose(offsetTransform_, resizeStartScale_, glm::quat(), glm::vec3(), glm::vec3(), glm::vec4());

        if (aButtonIsPressed) {
            isChangingProportions_ = true;
            isResizing_ = false;
            // Store panel axes at start of drag
            initialPanelXDir_ = glm::normalize(glm::vec3(transform[0]));
            initialPanelYDir_ = glm::normalize(glm::vec3(transform[1]));
            // Store the initial size to avoid jump at start of non-proportional resize
            resizeStartSize_ = size_;
        } else {
            isResizing_ = true;
            isChangingProportions_ = false;
                }
        }
    } else if (minimizeHit.didHit) {
            minimizeTargetState_ = !minimizeTargetState_;
            clickConsumed = true;
        } else if (closeHit.didHit) {
            isVisible_ = false;
            clickConsumed = true;
        }
    }
    
    // Only interact with main widgets if not minimized and no handle was clicked
    if (!clickConsumed && minimizeT_ < 0.99f) {
        IVRWidget* newHoveredWidget = nullptr;
        float closestHitDist = std::numeric_limits<float>::max();
        for (auto& widget : widgets_) {
            HitResult hit = widget->CheckIntersection(localRay);
            if (hit.didHit && hit.distance < closestHitDist) {
                closestHitDist = hit.distance;
                newHoveredWidget = widget.get();
            }
        }
        
        if (hoveredWidget_ != newHoveredWidget) {
            if (hoveredWidget_) hoveredWidget_->SetHover(false);
            hoveredWidget_ = newHoveredWidget;
            if (hoveredWidget_) hoveredWidget_->SetHover(true);
        }
    } else {
        if (hoveredWidget_) hoveredWidget_->SetHover(false);
        hoveredWidget_ = nullptr;
    }
    
    if (minimizeT_ < 0.99f) {
        for (auto& widget : widgets_) {
            widget->Update(localRay, false);
        }
    }
}

void VRPanel::Render(Urbaxio::Renderer& renderer, Urbaxio::TextRenderer& textRenderer, const glm::mat4& view, const glm::mat4& projection) {
    if (!isVisible_ || alpha < 0.01f) return;

    // --- Animation & Layout Calculations ---
    const float handleDiameter = 0.02f;
    const float handleSpacing = 0.025f;

    // 1. Determine handle positions based on panel state (minimized, narrow, or normal)
    glm::vec3 grabPos, minimizePos, closePos, resizePos;
    
    if (minimizeT_ > 0.99f) { // --- Minimized State ---
        const glm::vec2 minimizedSize(0.1f, 0.03f);
        // Place handles horizontally on top of the minimized stub
        closePos    = { minimizedSize.x * 0.5f - handleDiameter * 0.5f, minimizedSize.y * 0.5f + handleSpacing, 0.01f };
        minimizePos = closePos - glm::vec3(handleSpacing, 0, 0);
        grabPos     = minimizePos - glm::vec3(handleSpacing, 0, 0);
        resizePos   = { minimizedSize.x * 0.5f - handleDiameter * 0.5f, -minimizedSize.y * 0.5f + handleDiameter * 0.5f, 0.01f };
    } else { // --- Maximized or Animating State ---
        // Calculate the default horizontal positions first
        closePos    = { size_.x * 0.5f - handleDiameter * 0.5f, size_.y * 0.5f - handleDiameter * 0.5f, 0.01f };
        minimizePos = closePos - glm::vec3(handleSpacing, 0, 0);
        grabPos     = minimizePos - glm::vec3(handleSpacing, 0, 0);
        resizePos   = { size_.x * 0.5f - handleDiameter * 0.5f, -size_.y * 0.5f + handleDiameter * 0.5f, 0.01f };
        
        // Check if the grab handle would spill out
        bool useVerticalLayout = (grabPos.x - handleDiameter * 0.5f) < (-size_.x * 0.5f);

        if (useVerticalLayout) {
            // Switch to vertical layout, outside the top-left corner
            const float verticalOffset = 0.01f;
            closePos    = { -size_.x * 0.5f - verticalOffset, size_.y * 0.5f - handleDiameter * 0.5f, 0.01f };
            minimizePos = closePos - glm::vec3(0, handleSpacing, 0);
            grabPos     = minimizePos - glm::vec3(0, handleSpacing, 0);
            // keep resize in its default bottom-right corner for the current size
            resizePos   = { size_.x * 0.5f - handleDiameter * 0.5f, -size_.y * 0.5f + handleDiameter * 0.5f, 0.01f };
        }
    }

    // Apply the calculated positions to the handle widgets
    closeHandle_->SetLocalPosition(closePos);
    minimizeHandle_->SetLocalPosition(minimizePos);
    grabHandle_->SetLocalPosition(grabPos);
    resizeHandle_->SetLocalPosition(resizePos);

    // 2. Animate panel size and position for smooth minimizing
    const glm::vec2 minimizedSize(0.1f, 0.03f);
    glm::vec2 currentSize = glm::mix(size_, minimizedSize, minimizeT_);
    glm::vec3 positionOffset = glm::mix(
        glm::vec3(0.0f), 
        glm::vec3(size_.x * 0.5f - minimizedSize.x * 0.5f, size_.y * 0.5f - minimizedSize.y * 0.5f, 0.0f), 
        minimizeT_
    );
    float currentRadius = glm::mix(cornerRadius_, 0.5f, minimizeT_);
    float widgetsAlpha = alpha * (1.0f - minimizeT_);
    float minimizedTitleAlpha = alpha * minimizeT_;

    // 3. Render Background
    glm::mat4 backgroundModel = transform 
                              * glm::translate(glm::mat4(1.0f), positionOffset)
                              * glm::scale(glm::mat4(1.0f), glm::vec3(currentSize.x, currentSize.y, 1.0f));
    renderer.RenderVRPanel(view, projection, backgroundModel, glm::vec3(0.43f, 0.65f, 0.82f), currentRadius, 0.25f * alpha);
        
    // 4. Render Handles (they are now at their correct, dynamically calculated positions)
        grabHandle_->Render(renderer, textRenderer, transform, view, projection, alpha);
        minimizeHandle_->Render(renderer, textRenderer, transform, view, projection, alpha);
        closeHandle_->Render(renderer, textRenderer, transform, view, projection, alpha);
    resizeHandle_->Render(renderer, textRenderer, transform, view, projection, widgetsAlpha);

    // 5. Render Widgets or Minimized Title
        textRenderer.SetPanelModelMatrix(transform);
    if (widgetsAlpha > 0.01f) {
        for (auto& widget : widgets_) {
            widget->Render(renderer, textRenderer, transform, view, projection, widgetsAlpha);
        }
    }
    
    if (minimizedTitleAlpha > 0.01f) {
        // We use the 'grabPos' as an anchor for the title text's position
        glm::vec3 titlePos = grabPos - glm::vec3(0, handleSpacing, 0);
        textRenderer.AddTextOnPanel(displayName_, titlePos, glm::vec4(1.0f, 1.0f, 1.0f, minimizedTitleAlpha), 0.01f);
    }

    textRenderer.Render(view, projection);
}

HitResult VRPanel::CheckIntersection(const Ray& worldRay, const glm::mat4& parentTransform) {
    HitResult result;
    if (!isVisible_) return result;
    
    glm::mat4 finalTransform = parentTransform * offsetTransform_;
    glm::mat4 invFinalTransform = glm::inverse(finalTransform);
    Ray localRay;
    localRay.origin = invFinalTransform * glm::vec4(worldRay.origin, 1.0f);
    localRay.direction = glm::normalize(glm::vec3(invFinalTransform * glm::vec4(worldRay.direction, 0.0f)));
    
    // Always check handles first
    HitResult handleHits[] = {
        grabHandle_->CheckIntersection(localRay),
        resizeHandle_->CheckIntersection(localRay),
        minimizeHandle_->CheckIntersection(localRay),
        closeHandle_->CheckIntersection(localRay)
    };

    float closestHitDist = std::numeric_limits<float>::max();
    bool anyHandleHit = false;

    for(const auto& hit : handleHits) {
        if (hit.didHit && hit.distance < closestHitDist) {
            closestHitDist = hit.distance;
            result = hit;
            anyHandleHit = true;
        }
    }
    
    if (anyHandleHit) {
        result.distance = glm::distance(worldRay.origin, glm::vec3(finalTransform * glm::vec4(localRay.origin + localRay.direction * result.distance, 1.0)));
        return result;
    }

    // If no handle hit and panel is not minimized, check main panel area
    if (minimizeT_ < 0.99f) {
    glm::vec3 panelOrigin = glm::vec3(finalTransform[3]);
    glm::vec3 panelNormal = -glm::normalize(glm::vec3(finalTransform[2]));
    
    float t;
    if (glm::intersectRayPlane(worldRay.origin, worldRay.direction, panelOrigin, panelNormal, t) && t > 0) {
        glm::vec3 hitPoint = worldRay.origin + worldRay.direction * t;
        glm::vec3 localHitPoint = invFinalTransform * glm::vec4(hitPoint, 1.0f);
        if (glm::abs(localHitPoint.x) <= size_.x * 0.5f && glm::abs(localHitPoint.y) <= size_.y * 0.5f) {
            result.didHit = true;
            result.distance = t;
        }
    }
    }
    
    return result;
}

void VRPanel::SetVisible(bool visible) {
    isVisible_ = visible;
}

bool VRPanel::IsVisible() const {
    return isVisible_;
}

const std::string& VRPanel::GetName() const {
    return name_;
}

bool VRPanel::HandleClick() {
    if (hoveredWidget_) { 
        hoveredWidget_->HandleClick(); 
        return true; 
    }
    return false;
}

bool VRPanel::IsResizing() const {
    return isResizing_;
}

bool VRPanel::IsChangingProportions() const {
    return isChangingProportions_;
}

}

