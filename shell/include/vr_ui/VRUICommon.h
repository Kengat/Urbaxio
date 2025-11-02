// shell/include/vr_ui/VRUICommon.h
#pragma once

#include <glm/glm.hpp>
#include <string>

namespace Urbaxio::VRUI {

    // Struct to pass controller state to the UI system
    struct VRInputState {
        glm::vec3 rayOrigin;
        glm::vec3 rayDirection;
        bool triggerClicked = false;
        bool triggerReleased = false;
    };

    // Struct to hold styling information for all widgets
    struct VRUIStyle {
        glm::vec3 panelColor = {0.1f, 0.15f, 0.25f};
        float panelCornerRadius = 0.2f;
        float panelAlpha = 0.7f;

        glm::vec3 buttonColor = {0.2f, 0.3f, 0.5f};
        glm::vec3 buttonHoverColor = {0.3f, 0.45f, 0.75f};

        glm::vec4 textColor = {1.0f, 1.0f, 1.0f, 1.0f};
    };

}

