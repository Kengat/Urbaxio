#define GLM_ENABLE_EXPERIMENTAL
#include "ui/VRConfirmButtonWidget.h"
#include "renderer.h"
#include "TextRenderer.h"
#include <glm/gtx/intersect.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>

namespace Urbaxio::UI {

VRConfirmButtonWidget::VRConfirmButtonWidget(const glm::vec3& localPos, float diameter, glm::vec3 color, std::function<void()> onClick)
    : localPosition_(localPos), diameter_(diameter), color_(color), textureId_(0), onClick_(onClick) {}

VRConfirmButtonWidget::VRConfirmButtonWidget(const glm::vec3& localPos, float diameter, unsigned int textureId, std::function<void()> onClick, glm::vec3 color)
    : localPosition_(localPos), diameter_(diameter), color_(color), textureId_(textureId), onClick_(onClick) {}

void VRConfirmButtonWidget::setDepthEffect(DepthEffect effect) {

    effect_ = effect;

}

void VRConfirmButtonWidget::SetFadesWhenNotHovered(bool fades) {

    fadesWhenNotHovered_ = fades;

}

void VRConfirmButtonWidget::SetColor(const glm::vec3& newColor) { color_ = newColor; }

void VRConfirmButtonWidget::SetLabel(const std::string& text, bool alwaysVisible, float verticalOffset, float horizontalOffset) {
    labelText_ = text;
    labelAlwaysVisible_ = alwaysVisible;
    labelVerticalOffset_ = verticalOffset;
    labelHorizontalOffset_ = horizontalOffset;
}

void VRConfirmButtonWidget::Update(const Ray& localRay, bool triggerPressed, bool triggerReleased, bool triggerHeld, bool aButtonPressed, float stickY) {
    const float FADE_SPEED = 0.15f;
    float targetAlpha = isHovered_ ? 1.0f : 0.0f;
    hoverAlpha_ += (targetAlpha - hoverAlpha_) * FADE_SPEED;
}

void VRConfirmButtonWidget::HandleClick() {
    if (isHovered_ && onClick_) {
        onClick_();
    }
}

void VRConfirmButtonWidget::SetDepthStrength(float forwardFactorScale, float disparityScale) {
    forwardFactorScale_ = forwardFactorScale;
    disparityScale_ = disparityScale;
}

void VRConfirmButtonWidget::Render(Urbaxio::Renderer& renderer, Urbaxio::TextRenderer& textRenderer, const glm::mat4& panelTransform, const glm::mat4& view, const glm::mat4& projection, float alpha, const std::optional<MaskData>& mask) const {
    const glm::vec3 whiteColor(1.0f, 1.0f, 1.0f);
    const glm::vec3 closeButtonColor(1.00f, 0.20f, 0.32f);
    bool isCloseHandle = (glm::distance2(color_, closeButtonColor) < 0.01f);

    float finalAlpha;
    // START OF MODIFICATION
    // Use the new explicit flag to control visibility logic
    if (fadesWhenNotHovered_) {
        finalAlpha = hoverAlpha_ * alpha;
    } else {
        finalAlpha = alpha;
    }
    // END OF MODIFICATION

    if (finalAlpha < 0.01f && labelText_.empty()) {
        return;
    }
    
    float panelLocalScale = glm::length(glm::vec3(panelTransform[0]));
    float scaledDiameter = diameter_ * panelLocalScale;
    glm::vec3 worldPos = panelTransform * glm::vec4(localPosition_, 1.0f);

    // -- START OF MODIFICATION --

    // Spherical billboard logic using camera's UP vector to prevent flipping.

    glm::vec3 cameraPos = renderer.getCyclopsEyePosition();

    glm::vec3 cameraUp = glm::inverse(view)[1]; // Get camera's up vector in world space

    glm::mat4 lookAtMatrix = glm::lookAt(worldPos, cameraPos, cameraUp);

    glm::mat4 rotationMatrix = glm::inverse(lookAtMatrix);

    rotationMatrix[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f); // Remove translation component

    glm::mat4 modelMatrix = glm::translate(glm::mat4(1.0f), worldPos) *

                            rotationMatrix *

                            glm::scale(glm::mat4(1.0f), glm::vec3(scaledDiameter));

    // -- END OF MODIFICATION --
    
    float aberration = 0.05f + hoverAlpha_ * 0.10f;

    glm::vec3 abColor1;
    
    glm::vec3 abColor2;

    const glm::vec3 greenColor(0.1f, 0.8f, 0.2f);
    // --- НАЧАЛО ИЗМЕНЕНИЯ: Цвета абберации для синего и оранжевого ---
    const glm::vec3 blueColor(0.3f, 0.75f, 1.0f);
    const glm::vec3 orangeColor(1.0f, 0.79f, 0.4f);

    if (isCloseHandle) {
        abColor1 = glm::vec3(0.99f, 0.65f, 0.90f);
        abColor2 = glm::vec3(0.99f, 0.73f, 0.65f);
    } else if (glm::distance2(color_, greenColor) < 0.01f) {
        abColor1 = glm::vec3(0.55f, 1.00f, 0.18f);
        abColor2 = glm::vec3(0.00f, 1.00f, 0.75f);
    } else if (glm::distance2(color_, whiteColor) < 0.01f) {
        abColor1 = glm::vec3(0.93f, 0.72f, 1.00f);
        abColor2 = glm::vec3(0.7f, 0.9f, 1.0f);
    } else if (glm::distance2(color_, blueColor) < 0.01f) {
        abColor1 = glm::vec3(0.67f, 0.5f, 1.0f);
        abColor2 = glm::vec3(0.3f, 1.0f, 0.76f);
    } else if (glm::distance2(color_, orangeColor) < 0.01f) {
        abColor1 = glm::vec3(1.00f, 0.84f, 0.26f);
        abColor2 = glm::vec3(1.0f, 0.1f, 0.1f);
    } else { // Fallback to current color
        abColor1 = color_;
        abColor2 = color_;
    }
    // --- КОНЕЦ ИЗМЕНЕНИЯ ---
    
    if (finalAlpha > 0.01f) {
        renderer.RenderVRMenuWidget(view, projection, modelMatrix, color_, aberration, finalAlpha, abColor1, abColor2);
    }

    if (textureId_ != 0 && finalAlpha > 0.01f) {
        // -- START OF MODIFICATION --

        const float ICON_FORWARD_FACTOR_CONCAVE = 0.12f;  // Reduced concave forward offset

        const float ICON_FORWARD_FACTOR_CONVEX = -0.05f;  // Reduced convex backward offset

        const float DISPARITY_OFFSET = 0.02f;             // Reduced stereo disparity

        glm::vec3 z_axis = glm::vec3(rotationMatrix[2]);

        glm::vec3 iconWorldPos;

        

        int eyeIndex = renderer.getCurrentEyeIndex();

        float disparity_x_offset;

        // Use base diameter (not scaled) for offsets to maintain proportional effect regardless of scale
        if (effect_ == DepthEffect::CONVEX) {

            iconWorldPos = worldPos + z_axis * (diameter_ * ICON_FORWARD_FACTOR_CONVEX * forwardFactorScale_);

            // For CONVEX (popped out): left eye sees right (+), right eye sees left (-)

            disparity_x_offset = ((eyeIndex == 0) ? DISPARITY_OFFSET : -DISPARITY_OFFSET) * disparityScale_;

        } else { // CONCAVE effect

            iconWorldPos = worldPos + z_axis * (diameter_ * ICON_FORWARD_FACTOR_CONCAVE * forwardFactorScale_);

            // For CONCAVE (pushed in): left eye sees left (-), right eye sees right (+)

            disparity_x_offset = ((eyeIndex == 0) ? -DISPARITY_OFFSET : DISPARITY_OFFSET) * disparityScale_;

        }

        glm::vec3 cameraRight = glm::inverse(view)[0];

        iconWorldPos += cameraRight * (diameter_ * disparity_x_offset);

        

        glm::mat4 iconModelMatrix = glm::translate(glm::mat4(1.0f), iconWorldPos) *

                                    rotationMatrix *

                                    glm::scale(glm::mat4(1.0f), glm::vec3(scaledDiameter * 0.8f));

        // -- END OF MODIFICATION --

        renderer.RenderVRMenuWidget(view, projection, iconModelMatrix, glm::vec3(1.0f), 0.0f, finalAlpha, glm::vec3(0.0f), glm::vec3(0.0f), textureId_);
    }

    if (!labelText_.empty()) {
        float textAlpha = alpha;
        if (!labelAlwaysVisible_) {
            textAlpha *= hoverAlpha_;
        }

        if (textAlpha > 0.01f) {
            textRenderer.SetPanelModelMatrix(panelTransform);
            glm::vec3 textPos = localPosition_;
            textPos.z += 0.005f; // Reduced from 0.02f to push text back closer to panel surface
            textPos.y += labelVerticalOffset_;
            textPos.x += labelHorizontalOffset_;
            float textHeight = diameter_ * 0.6f;
            glm::vec4 textColor(1.0f, 1.0f, 1.0f, textAlpha);
            textRenderer.AddTextOnPanel(labelText_, textPos, textColor, textHeight, Urbaxio::TextAlign::CENTER, mask);
        }
    }
}

HitResult VRConfirmButtonWidget::CheckIntersection(const Ray& localRay) {
    HitResult result;
    float t;
    float radius = diameter_ * 0.5f;

    if (glm::intersectRaySphere(localRay.origin, localRay.direction, localPosition_, radius * radius, t) && t > 0) {
        result.didHit = true;
        result.distance = t;
        result.hitWidget = this;
    }
    return result;
}

void VRConfirmButtonWidget::SetLocalPosition(const glm::vec3& pos) { localPosition_ = pos; }

const glm::vec3& VRConfirmButtonWidget::GetLocalPosition() const { return localPosition_; }

void VRConfirmButtonWidget::SetSize(const glm::vec2& size) {
    // For a sphere in a vertical layout, the height is the most relevant dimension for diameter.
    diameter_ = size.y;
}

glm::vec2 VRConfirmButtonWidget::GetSize() const { return {diameter_, diameter_}; }

} // namespace Urbaxio::UI

