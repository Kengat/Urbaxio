#pragma once

#include "ui/IVRWidget.h"
#include <vector>
#include <memory>
#include <string>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace Urbaxio {
    class Renderer;
    class TextRenderer;
}

namespace Urbaxio::UI {

class VRPanel {
public:
    VRPanel(const std::string& name, const glm::vec2& size, const glm::mat4& offsetTransform);

    void AddWidget(std::unique_ptr<IVRWidget> widget);
    void Update(const Ray& worldRay, const glm::mat4& parentTransform, bool isClicked);
    void Render(Renderer& renderer, TextRenderer& textRenderer, const glm::mat4& view, const glm::mat4& projection);
    HitResult CheckIntersection(const Ray& worldRay, const glm::mat4& parentTransform);
    bool HandleClick();

    void SetVisible(bool visible);
    bool IsVisible() const;
    const std::string& GetName() const;
    
    glm::mat4 transform;
    float alpha = 0.0f;

    bool isGrabbing = false; // This will be managed from main.cpp
    bool isHoveringGrabHandle = false;
    glm::mat4 grabbedInitialTransform;
    glm::mat4 grabbedControllerInitialTransform;
    glm::mat4& GetOffsetTransform() { return offsetTransform_; }

private:
    std::string name_;
    glm::vec2 size_;
    glm::mat4 offsetTransform_;
    bool isVisible_ = true;
    
    IVRWidget* hoveredWidget_ = nullptr;
    std::vector<std::unique_ptr<IVRWidget>> widgets_;
    
    glm::vec3 GetGrabHandleCenter();
};

}

