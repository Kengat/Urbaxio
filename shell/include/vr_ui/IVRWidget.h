// shell/include/vr_ui/IVRWidget.h
#pragma once

#include "vr_ui/VRUICommon.h"
#include <memory>

// Forward declarations
namespace Urbaxio { class Renderer; class TextRenderer; }

namespace Urbaxio::VRUI {

class IVRWidget {
public:
    virtual ~IVRWidget() = default;

    // Handle input and update internal state
    virtual void Update(const VRInputState& input, const VRUIStyle& style) = 0;

    // Draw the widget
    virtual void Render(Renderer& renderer, TextRenderer& textRenderer, const glm::mat4& view, const glm::mat4& projection, const VRUIStyle& style) = 0;
    
    // Set the parent transform that will be applied to this widget
    virtual void SetParentTransform(const glm::mat4& parentTransform) = 0;

    // --- Methods for interaction required by UIManager ---
    
    // Set the hover state from the manager (after it determines the closest hit)
    virtual void SetHovered(bool hovered) = 0;
    // Perform a hit test against the widget and return distance along the ray, or -1 if no hit.
    virtual float CheckHit(const VRInputState& input) = 0;
};

}

