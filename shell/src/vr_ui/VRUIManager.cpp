// shell/src/vr_ui/VRUIManager.cpp
// --- FIX: Enable experimental GLM features ---
#define GLM_ENABLE_EXPERIMENTAL
#include "vr_ui/VRTextButton.h" // <-- NEW: Include our button widget
#include "vr_ui/VRTextDisplay.h"
#include "vr_ui/VRBillboardButton.h"
#include <SDL2/SDL_keycode.h>

#include "vr_ui/VRUIManager.h"
#include "vr_ui/VRPanel.h"
#include "VRManager.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include <SDL2/SDL.h>
#include <limits>

namespace Urbaxio::VRUI {

VRUIManager::VRUIManager() {}

// --- FIX: Define destructor here where VRPanel is a complete type ---
VRUIManager::~VRUIManager() = default;

void VRUIManager::Initialize(
    Urbaxio::VRManager* vrManager, 
    std::string* numpadInputString,
    std::function<void(SDL_Keycode)> onNumpadKey,
    bool* isNumpadActive
) {
    vrManager_ = vrManager;
    isNumpadActive_ = isNumpadActive;
    CreateNumpadPanel(numpadInputString, onNumpadKey, isNumpadActive);
}

void VRUIManager::CreateNumpadPanel(std::string* numpadInput, std::function<void(SDL_Keycode)> onNumpadKey, bool* isNumpadActive) {
    auto numpadPanel = std::make_unique<VRPanel>("Numpad");
    
    float panelScale = 0.450f;
    glm::vec3 translation = glm::vec3(-0.024f, -0.047f, -0.197f);
    glm::vec3 eulerAnglesRad = glm::radians(glm::vec3(-103.003f, 4.477f, -14.612f));
    glm::mat4 numpadOffsetTransform =
        glm::translate(glm::mat4(1.0f), translation) *
        glm::mat4_cast(glm::quat(eulerAnglesRad)) *
        glm::scale(glm::mat4(1.0f), glm::vec3(panelScale));
    
    numpadPanel->SetOffsetTransform(numpadOffsetTransform);

    glm::mat4 displayTransform = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.06f, 0.0f));
    displayTransform = glm::scale(displayTransform, glm::vec3(0.2f, 0.05f, 1.0f));
    auto display = std::make_unique<VRTextDisplay>(numpadInput, displayTransform);
    numpadPanel->AddWidget(std::move(display));

    const char* numpadKeys[12] = {"1","2","3", "4","5","6", "7","8","9", ".","0","<-"};
    float keySpacing = 0.06f;
    float keyRadius = 0.028f;

    for (int i = 0; i < 12; ++i) {
        glm::vec3 keyCenter(0.0f);
        int row = i / 3; int col = i % 3;

        if (i < 9) { keyCenter = glm::vec3(((float)col - 1.0f) * keySpacing, -((float)row * keySpacing), 0.0f); }
        else if (i == 9) { keyCenter = glm::vec3(-keySpacing, -3.0f * keySpacing, 0.0f); }
        else if (i == 10) { keyCenter = glm::vec3(0.0f, -3.0f * keySpacing, 0.0f); }
        else if (i == 11) { keyCenter = glm::vec3(keySpacing, -3.0f * keySpacing, 0.0f); }

        auto button = std::make_unique<VRTextButton>(numpadKeys[i], glm::translate(glm::mat4(1.0f), keyCenter), keyRadius);
        
        if (i < 9) {
            button->OnClick = [numpadInput, i]() { if (*numpadInput == "0") *numpadInput = ""; *numpadInput += std::to_string(i + 1); };
        } else if (i == 9) {
            button->OnClick = [numpadInput]() { if (numpadInput->find('.') == std::string::npos) *numpadInput += '.'; };
        } else if (i == 10) {
            button->OnClick = [numpadInput]() { if (*numpadInput != "0") *numpadInput += '0'; };
        } else if (i == 11) {
            button->OnClick = [numpadInput]() { if (numpadInput->length() > 1) numpadInput->pop_back(); else *numpadInput = "0"; };
        }

        numpadPanel->AddWidget(std::move(button));
    }

    glm::vec3 confirmCenter = glm::vec3(0.0f, -4.0f * keySpacing, 0.0f);
    auto confirmButton = std::make_unique<VRBillboardButton>(glm::translate(glm::mat4(1.0f), confirmCenter), 0.018f);
    confirmButton->baseColor = {0.1f, 0.8f, 0.2f};
    confirmButton->OnClick = [onNumpadKey, isNumpadActive]() {
        onNumpadKey(SDLK_RETURN);
        *isNumpadActive = false;
    };
    numpadPanel->AddWidget(std::move(confirmButton));

    glm::vec3 grabHandleCenter = glm::vec3(0.0f, 0.06f, 0.0f) + glm::vec3(0.0f, 0.05f * 0.5f + 0.012f + 0.01f, 0.0f);
    auto grabHandle = std::make_unique<VRBillboardButton>(glm::translate(glm::mat4(1.0f), grabHandleCenter), 0.014f);
    grabHandle->baseColor = {1.0f, 1.0f, 1.0f};
    VRPanel* panel_ptr = numpadPanel.get();
    grabHandle->OnPressStateChanged = [this, panel_ptr](bool isPressed) {
        if (isPressed) {
            const auto& rightHand = vrManager_->GetRightHandVisual();
            if (rightHand.isValid) {
                 glm::mat4 rawPoseMatrix = glm::toMat4(glm::quat(rightHand.pose.orientation.w, rightHand.pose.orientation.x, rightHand.pose.orientation.y, rightHand.pose.orientation.z));
                 rawPoseMatrix[3] = glm::vec4(rightHand.pose.position.x, rightHand.pose.position.y, rightHand.pose.position.z, 1.0);
                 glm::mat4 grabControllerTransform = vrManager_->GetWorldTransform() * rawPoseMatrix;
                 panel_ptr->StartDrag(grabControllerTransform);
            }
        } else {
            panel_ptr->StopDrag();
        }
    };
    numpadPanel->AddWidget(std::move(grabHandle));

    panels_[numpadPanel->GetName()] = std::move(numpadPanel);
}

void VRUIManager::Update(const VRInputState& input, const glm::mat4& rightHandTransform) {
    closestHitDistance_ = -1.0f;
    if (!vrManager_ || !vrManager_->IsInitialized() || !*isNumpadActive_) return;
    
    const auto& leftHand = vrManager_->GetLeftHandVisual();
    if (!leftHand.isValid) return;
    
    glm::mat4 rawPoseMatrix = glm::toMat4(glm::quat(leftHand.pose.orientation.w, leftHand.pose.orientation.x, leftHand.pose.orientation.y, leftHand.pose.orientation.z));
    rawPoseMatrix[3] = glm::vec4(leftHand.pose.position.x, leftHand.pose.position.y, leftHand.pose.position.z, 1.0);
    glm::mat4 parentTransform = vrManager_->GetWorldTransform() * rawPoseMatrix;
    
    IVRWidget* closestHoveredWidget = nullptr;
    float minHitDistance = std::numeric_limits<float>::max();

    for (auto& [name, panel] : panels_) {
        panel->SetParentTransform(parentTransform);
        panel->UpdateDrag(rightHandTransform);
        
        for (auto* widget : panel->GetAllWidgets()) {
            widget->SetParentTransform(panel->GetFinalTransform());
            float hitDist = widget->CheckHit(input);
            if (hitDist >= 0.0f && hitDist < minHitDistance) {
                minHitDistance = hitDist;
                closestHoveredWidget = widget;
            }
        }
    }
    closestHitDistance_ = minHitDistance;

    for (auto& [name, panel] : panels_) {
        for (auto* widget : panel->GetAllWidgets()) {
            widget->SetHovered(widget == closestHoveredWidget);
            widget->Update(input, style_);
        }
    }
}

float VRUIManager::GetClosestHitDistance() const {
    return closestHitDistance_;
}

void VRUIManager::RenderAll(Renderer& renderer, TextRenderer& textRenderer, const glm::mat4& view, const glm::mat4& projection) {
    if (!vrManager_ || vrManager_->leftMenuAlpha < 0.01f || !(*isNumpadActive_)) return;

    for (auto& [name, panel] : panels_) {
        panel->Render(renderer, textRenderer, view, projection, style_);
    }
}

}

