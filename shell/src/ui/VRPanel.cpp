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
#include <algorithm>
#include <vector>

namespace Urbaxio::UI {

VRPanel::VRPanel(const std::string& name, const std::string& displayName, const glm::vec2& size, const glm::mat4& offsetTransform, float cornerRadius, unsigned int grabIcon, unsigned int closeIcon, unsigned int minimizeIcon)
    : name_(name), displayName_(displayName), initialOffsetTransform_(offsetTransform), size_(size), offsetTransform_(offsetTransform), cornerRadius_(cornerRadius), hoveredWidget_(nullptr), isGrabbing(false) {
    
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
    // START OF MODIFICATION
    resizeHandle_->SetFadesWhenNotHovered(true);
    // END OF MODIFICATION
    // -- START OF MODIFICATION --

    // Set the desired depth effect for the panel control buttons to CONVEX (popped out)

    closeHandle_->setDepthEffect(DepthEffect::CONVEX);

    minimizeHandle_->setDepthEffect(DepthEffect::CONVEX);

    grabHandle_->setDepthEffect(DepthEffect::CONVEX);

    // Сделать у мелких кнопок панели эффект глубины менее выраженным
    closeHandle_->SetDepthStrength(0.5f, 0.5f);
    minimizeHandle_->SetDepthStrength(0.5f, 0.5f);
    grabHandle_->SetDepthStrength(0.5f, 0.5f);

    // -- END OF MODIFICATION --

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

void VRPanel::Update(const Ray& worldRay, const glm::mat4& parentTransform, const glm::mat4& interactionTransform, bool triggerPressed, bool triggerReleased, bool triggerHeld, bool aButtonPressed, bool aButtonHeld, bool bButtonIsPressed, float stickY, bool isLeftTriggerPressed) {
    // --- НОВАЯ ЛОГИКА АНИМАЦИИ ALPHA ---
    const float ANIM_SPEED = 0.1f;
    float targetAlpha = 0.0f;

    if (isVisible_) {
        switch (visibilityMode_) {
            case VisibilityMode::ALWAYS_VISIBLE:
                targetAlpha = 1.0f;
                break;
            case VisibilityMode::ON_LEFT_TRIGGER:
                targetAlpha = isLeftTriggerPressed ? 1.0f : 0.0f;
                break;
            case VisibilityMode::TOGGLE_VIA_FLAG:
                targetAlpha = (isExternallyToggled_ && isLeftTriggerPressed) ? 1.0f : 0.0f;
                break;
        }
    }
    
    alpha += (targetAlpha - alpha) * ANIM_SPEED;
    alpha = std::min(1.0f, std::max(0.0f, alpha));
    // --- КОНЕЦ НОВОЙ ЛОГИКИ ---

    // --- Animation Logic ---
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
        if (!triggerHeld) {
            isResizing_ = false;
            isChangingProportions_ = false;
        } else {
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
                // -- START OF MODIFICATION --
                // Compensate for the global world scale
                float worldScale = glm::length(glm::vec3(parentTransform[0]));
                if (worldScale < 1e-5f) worldScale = 1.0f;

                // Vectors from panel center to current and start controller positions (on panel plane)
                glm::vec3 currentVec = controllerPosOnPlane - panelCenterAtResizeStart_;
                glm::vec3 initialVec = resizeStartControllerPos_ - panelCenterAtResizeStart_;
                
                // Absolute distances along panel axes captured at drag start (world units)
                float currentDistX = std::abs(glm::dot(currentVec, initialPanelXDir_));
                float currentDistY = std::abs(glm::dot(currentVec, initialPanelYDir_));
                float initialDistX = std::abs(glm::dot(initialVec, initialPanelXDir_));
                float initialDistY = std::abs(glm::dot(initialVec, initialPanelYDir_));

                // Change in half-extent along each axis (in world units)
                float deltaX_world = currentDistX - initialDistX;
                float deltaY_world = currentDistY - initialDistY;

                // Convert world delta to the controller's local space (unscaled by world scale)
                float deltaX_local = deltaX_world / worldScale;
                float deltaY_local = deltaY_world / worldScale;

                // Compensate for the panel's own local scale to get the delta for the 'size_' property
                float scaleCompensatorX = (resizeStartScale_.x > 1e-5f) ? (1.0f / resizeStartScale_.x) : 1.0f;
                float scaleCompensatorY = (resizeStartScale_.y > 1e-5f) ? (1.0f / resizeStartScale_.y) : 1.0f;

                // New size is start size plus twice the compensated delta (panel grows symmetrically)
                const float minSize = 0.05f;
                float newWidth  = resizeStartSize_.x + (deltaX_local * 2.0f) * scaleCompensatorX;
                float newHeight = resizeStartSize_.y + (deltaY_local * 2.0f) * scaleCompensatorY;
                size_.x = glm::max(minSize, newWidth);
                size_.y = glm::max(minSize, newHeight);
                // -- END OF MODIFICATION --

                RecalculateLayout();
            }
        }
    }

    if (triggerReleased) {
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
    
    grabHandle_->Update(localRay, triggerPressed && grabHit.didHit, triggerReleased, triggerHeld, aButtonPressed, 0.0f);
    resizeHandle_->Update(localRay, triggerPressed && resizeHit.didHit, triggerReleased, triggerHeld, aButtonPressed, 0.0f);
    minimizeHandle_->Update(localRay, (triggerPressed || aButtonPressed) && minimizeHit.didHit, triggerReleased, triggerHeld, aButtonPressed, 0.0f);
    closeHandle_->Update(localRay, (triggerPressed || aButtonPressed) && closeHit.didHit, triggerReleased, triggerHeld, aButtonPressed, 0.0f);

    if (triggerPressed) {
        // Handle logging first to ensure we read final size_ from previous frame's resize
        if (resizeHit.didHit && bButtonIsPressed) {
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
            clickConsumed = true;
        } else if (grabHit.didHit) {
            isGrabbing = true;
            grabbedInitialTransform = transform;
            grabbedControllerInitialTransform = interactionTransform;
            clickConsumed = true;
        } else if (resizeHit.didHit) {
            clickConsumed = true;
            // --- START RESIZE OR PROPORTION CHANGE ---
            panelCenterAtResizeStart_ = glm::vec3(transform[3]);
            
            // Project start point onto panel plane
            glm::vec3 panelNormal = -glm::normalize(glm::vec3(transform[2]));
            float t;
            glm::intersectRayPlane(worldRay.origin, worldRay.direction, panelCenterAtResizeStart_, panelNormal, t);
            resizeStartControllerPos_ = worldRay.origin + worldRay.direction * t;
            lastControllerPosOnPlane_ = resizeStartControllerPos_;

            // Store initial scale
            (void)glm::decompose(offsetTransform_, resizeStartScale_, glm::quat(), glm::vec3(), glm::vec3(), glm::vec4());

            // Non-proportional resize now requires BOTH trigger and A-button to be held
            if (aButtonHeld) {
                isChangingProportions_ = true;
                isResizing_ = false;
                // Store panel axes at start of drag
                initialPanelXDir_ = glm::normalize(glm::vec3(transform[0]));
                initialPanelYDir_ = glm::normalize(glm::vec3(transform[1]));
                // Store the initial size to avoid jump at start of non-proportional resize
                resizeStartSize_ = size_;
            } else { // Proportional resize is the default for trigger
                isResizing_ = true;
                isChangingProportions_ = false;
            }
        }
    }

    // Handle simple clicks (Trigger or A button) for close/minimize
    if ((triggerPressed || aButtonPressed) && !clickConsumed) {
        if (minimizeHit.didHit) {
            SetMinimized(!IsMinimized());
            clickConsumed = true;
        } else if (closeHit.didHit) {
            SetVisible(false);
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
    
    // --- START OF MODIFICATION ---
    if (minimizeT_ < 0.99f) {
        for (auto& widget : widgets_) {
            bool childIsHovered = (hoveredWidget_ == widget.get());
            // Pass triggerPressed and aButtonPressed separately, gated by hover.
            widget->Update(localRay, triggerPressed && childIsHovered, triggerReleased, triggerHeld, aButtonPressed && childIsHovered, stickY);
        }
    }
    // --- END OF MODIFICATION ---
}

void VRPanel::Render(Urbaxio::Renderer& renderer, Urbaxio::TextRenderer& textRenderer, const glm::mat4& view, const glm::mat4& projection) const {
    if (!isVisible_ || alpha < 0.01f) return;

    // --- Animation & Layout Constants ---
    const float handleDiameter = 0.02f;
    const float handleSpacing = 0.025f;
    const glm::vec2 minimizedSize(0.1f, 0.03f);

    // --- 1. Calculate Animated Panel Geometry ---
    glm::vec2 currentSize = glm::mix(size_, minimizedSize, minimizeT_);
    glm::vec3 positionOffset = glm::mix(
        glm::vec3(0.0f), 
        glm::vec3(size_.x * 0.5f - minimizedSize.x * 0.5f, size_.y * 0.5f - minimizedSize.y * 0.5f, 0.0f), 
        minimizeT_
    );
    float currentRadius = glm::mix(cornerRadius_, 0.5f, minimizeT_);
    float widgetsAlpha = alpha * (1.0f - minimizeT_);
    float minimizedTitleAlpha = alpha * minimizeT_;

    // --- 2. Calculate Handle Positions with Smooth Interpolation ---
    glm::vec3 grabPos, minimizePos, closePos;

    // Define the two possible layouts for the MAXIMIZED state
    glm::vec3 h_closePos    = { size_.x * 0.5f - handleDiameter * 0.5f, size_.y * 0.5f - handleDiameter * 0.5f, 0.01f };
    glm::vec3 h_minimizePos = h_closePos - glm::vec3(handleSpacing, 0, 0);
    glm::vec3 h_grabPos     = h_minimizePos - glm::vec3(handleSpacing, 0, 0);

    const float verticalOffset = 0.01f;
    glm::vec3 v_closePos    = { -size_.x * 0.5f - verticalOffset, size_.y * 0.5f - handleDiameter * 0.5f, 0.01f };
    glm::vec3 v_minimizePos = v_closePos - glm::vec3(0, handleSpacing, 0);
    glm::vec3 v_grabPos     = v_minimizePos - glm::vec3(0, handleSpacing, 0);

    // Define the single layout for the MINIMIZED state
    glm::vec3 m_closePos    = { minimizedSize.x * 0.5f - handleDiameter * 0.5f, minimizedSize.y * 0.5f, 0.01f };
    m_closePos += positionOffset; // Adjust to the corner of the moved stub
    glm::vec3 m_minimizePos = m_closePos - glm::vec3(handleSpacing, 0, 0);
    glm::vec3 m_grabPos     = m_minimizePos - glm::vec3(handleSpacing, 0, 0);
    
    // Decide which layout to use for the maximized state
    bool useVerticalLayout = (h_grabPos.x - handleDiameter * 0.5f) < (-size_.x * 0.5f);
    
    // Interpolate positions from the chosen maximized layout to the minimized layout
    closePos    = glm::mix(useVerticalLayout ? v_closePos    : h_closePos,    m_closePos,    minimizeT_);
    minimizePos = glm::mix(useVerticalLayout ? v_minimizePos : h_minimizePos, m_minimizePos, minimizeT_);
    grabPos     = glm::mix(useVerticalLayout ? v_grabPos     : h_grabPos,     m_grabPos,     minimizeT_);
    
    // The resize handle just moves with the corner and fades out
    glm::vec3 resizePos = { size_.x * 0.5f - handleDiameter * 0.5f, -size_.y * 0.5f + handleDiameter * 0.5f, 0.01f };

    // Apply final positions
    closeHandle_->SetLocalPosition(closePos);
    minimizeHandle_->SetLocalPosition(minimizePos);
    grabHandle_->SetLocalPosition(grabPos);
    resizeHandle_->SetLocalPosition(resizePos);

    // --- 3. Render Background ---
    glm::mat4 backgroundModel = transform 
                              * glm::translate(glm::mat4(1.0f), positionOffset)
                              * glm::scale(glm::mat4(1.0f), glm::vec3(currentSize.x, currentSize.y, 1.0f));
    renderer.RenderVRPanel(view, projection, backgroundModel, glm::vec3(0.43f, 0.65f, 0.82f), currentRadius, 0.25f * alpha);
    
    // --- 4. Собираем ВСЕ виджеты (основные и служебные) для сортировки ---
    std::vector<IVRWidget*> allWidgetsToRender;
    // Добавляем служебные виджеты
    allWidgetsToRender.push_back(grabHandle_.get());
    allWidgetsToRender.push_back(minimizeHandle_.get());
    allWidgetsToRender.push_back(closeHandle_.get());
    // Виджет изменения размера рендерится только в развёрнутом состоянии
    if (widgetsAlpha > 0.01f) {
        allWidgetsToRender.push_back(resizeHandle_.get());
    }
    // Добавляем основные виджеты, если панель не свёрнута
    if (widgetsAlpha > 0.01f) {
        for (const auto& widget : widgets_) {
            allWidgetsToRender.push_back(widget.get());
        }
    }

    // --- 5. Сортируем все виджеты по глубине ---
    std::vector<std::pair<float, IVRWidget*>> order;
    order.reserve(allWidgetsToRender.size());
    for (IVRWidget* widget : allWidgetsToRender) {
        const glm::vec3 local = widget->GetLocalPosition();
        const glm::vec3 world = glm::vec3(transform * glm::vec4(local, 1.0f)); 
        const float viewZ = (view * glm::vec4(world, 1.0f)).z;
        order.emplace_back(viewZ, widget);
    }

    std::sort(order.begin(), order.end(), [](const auto& a, const auto& b){ 
        return a.first < b.first; 
    });

    // --- 6. Рендерим все виджеты в отсортированном порядке ---
    textRenderer.SetPanelModelMatrix(transform);
    for (const auto& [viewZ, widget] : order) {
        // Определяем альфу для виджета. resizeHandle_ и основные виджеты исчезают при сворачивании.
        float currentWidgetAlpha = (widget == resizeHandle_.get() || std::find_if(widgets_.begin(), widgets_.end(), [widget](const auto& p){ return p.get() == widget; }) != widgets_.end())
                                 ? widgetsAlpha
                                 : alpha;
        
        widget->Render(renderer, textRenderer, transform, view, projection, currentWidgetAlpha, std::nullopt);
    }
    
    // --- 7. Рендерим свёрнутый заголовок, если нужно ---
    if (minimizedTitleAlpha > 0.01f) {
        glm::vec3 titlePos = grabPos - glm::vec3(0, handleSpacing, 0);
        textRenderer.AddTextOnPanel(displayName_, titlePos, glm::vec4(1.0f, 1.0f, 1.0f, minimizedTitleAlpha), 0.01f, Urbaxio::TextAlign::CENTER, std::nullopt);
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
    if (visibilityMode_ == VisibilityMode::TOGGLE_VIA_FLAG) {
        isExternallyToggled_ = visible;
    }
    isVisible_ = visible;
}

bool VRPanel::IsVisible() const {
    return isVisible_;
}

void VRPanel::SetVisibilityMode(VisibilityMode mode) {
    visibilityMode_ = mode;
}

VisibilityMode VRPanel::GetVisibilityMode() const {
    return visibilityMode_;
}

const std::string& VRPanel::GetName() const {
    return name_;
}

const std::string& VRPanel::GetDisplayName() const {
    return displayName_;
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

void VRPanel::SetMinimized(bool minimized) {
    minimizeTargetState_ = minimized;
}

bool VRPanel::IsMinimized() const {
    return minimizeTargetState_;
}

void VRPanel::ResetPosition() {
    offsetTransform_ = initialOffsetTransform_;
}

}

