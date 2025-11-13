#pragma once

#include "ui/IVRWidget.h"
#include "ui/Layouts.h"
#include "ui/VRConfirmButtonWidget.h"
#include <vector>
#include <memory>
#include <string>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

namespace Urbaxio {
    class Renderer;
    class TextRenderer;
}

namespace Urbaxio::UI {

enum class VisibilityMode {
    ALWAYS_VISIBLE,    // Всегда видна (если isVisible_ = true)
    ON_LEFT_TRIGGER,   // Появляется при зажатии левого курка
    TOGGLE_VIA_FLAG    // Видимость переключается внешним флагом (для PanelManager)
};

class VRPanel {
public:
    VRPanel(const std::string& name, const std::string& displayName, const glm::vec2& size, const glm::mat4& offsetTransform, float cornerRadius, unsigned int grabIcon, unsigned int pinIcon, unsigned int closeIcon, unsigned int minimizeIcon);

    void AddWidget(std::unique_ptr<IVRWidget> widget);
    void SetMinimized(bool minimized);
    bool IsMinimized() const;
    void ResetPosition();

    void SetLayout(std::unique_ptr<ILayout> layout);
    void RecalculateLayout();
    void Update(const Ray& worldRay, const glm::mat4& parentTransform, const glm::mat4& interactionTransform, bool triggerPressed, bool triggerReleased, bool triggerHeld, bool aButtonPressed, bool aButtonHeld, bool bButtonIsPressed, float stickY, bool isLeftTriggerPressed);
    void Render(Renderer& renderer, TextRenderer& textRenderer, const glm::mat4& view, const glm::mat4& projection) const;
    HitResult CheckIntersection(const Ray& worldRay, const glm::mat4& parentTransform) const;
    bool HandleClick();

    void SetVisible(bool visible);
    bool IsVisible() const;
    const std::string& GetName() const;
    const std::string& GetDisplayName() const;
    
    void SetVisibilityMode(VisibilityMode mode);
    VisibilityMode GetVisibilityMode() const;
    
    IVRWidget* GetHoveredWidget() const { return hoveredWidget_; }
    IVRWidget* GetWidget(size_t index) const { return (index < widgets_.size()) ? widgets_[index].get() : nullptr; }
    
    bool IsResizing() const;
    bool IsChangingProportions() const;
    void SetPinned(bool pinned, const glm::mat4& currentParentTransform, const glm::mat4& newParentTransform);
    bool IsPinned() const;
    bool WasPinButtonClicked();
    
    void OnSizeChanged();
    
    glm::mat4 transform;
    float alpha = 0.0f;

    bool isGrabbing = false; // This will be managed from main.cpp
    bool isHoveringGrabHandle = false;
    glm::mat4 grabbedInitialTransform;
    glm::mat4 grabbedControllerInitialTransform;
    float grabHandleHoverAlpha_ = 0.0f;
    glm::mat4& GetOffsetTransform() { return offsetTransform_; }

private:
    std::string name_;
    std::string displayName_;
    glm::mat4 initialOffsetTransform_;
    glm::vec2 size_;
    glm::mat4 offsetTransform_;
    bool isVisible_ = true;
    float cornerRadius_;
    VisibilityMode visibilityMode_ = VisibilityMode::ON_LEFT_TRIGGER;
    bool isExternallyToggled_ = false;
    std::unique_ptr<ILayout> layout_;
    
    std::unique_ptr<VRConfirmButtonWidget> grabHandle_;
    std::unique_ptr<VRConfirmButtonWidget> resizeHandle_;
    std::unique_ptr<VRConfirmButtonWidget> minimizeHandle_;
    std::unique_ptr<VRConfirmButtonWidget> closeHandle_;
    std::unique_ptr<VRConfirmButtonWidget> pinHandle_;
    bool isPinned_ = false;
    float serviceButtonsAlpha_ = 0.0f;
    VisibilityMode pinnedVisibilityBackup_ = VisibilityMode::ON_LEFT_TRIGGER;
    bool hasPinnedVisibilityBackup_ = false;
    bool pinButtonClicked_ = false;

    bool isResizing_ = false;
    bool isChangingProportions_ = false;
    bool minimizeTargetState_ = false;
    float minimizeT_ = 0.0f;
    glm::vec3 initialPanelXDir_{1.0f, 0.0f, 0.0f};
    glm::vec3 initialPanelYDir_{0.0f, 1.0f, 0.0f};
    glm::vec3 resizeStartControllerPos_{0.0f};
    glm::vec3 resizeStartScale_{1.0f};
    glm::vec3 panelCenterAtResizeStart_{0.0f};
    glm::vec2 resizeStartSize_{1.0f};
    glm::vec3 lastControllerPosOnPlane_{0.0f};
    
    IVRWidget* hoveredWidget_ = nullptr;
    std::vector<std::unique_ptr<IVRWidget>> widgets_;
};

}

