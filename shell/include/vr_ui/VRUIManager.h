// shell/include/vr_ui/VRUIManager.h
#pragma once

#include "vr_ui/VRUICommon.h"
#include <map>
#include <string>
#include <memory>
#include <functional>

// Forward declarations
namespace Urbaxio { class Renderer; class TextRenderer; class VRManager; }
namespace Urbaxio::VRUI { class VRPanel; }

#include <SDL2/SDL_keycode.h>
#include <functional>

namespace Urbaxio::VRUI {

class VRUIManager {
public:
    VRUIManager();
    ~VRUIManager();

    void Initialize(
        Urbaxio::VRManager* vrManager,
        std::string* numpadInputString,
        std::function<void(SDL_Keycode)> onNumpadKey,
        bool* isNumpadActive
    );
    
    void Update(const VRInputState& input, const glm::mat4& rightHandTransform);
    void RenderAll(Renderer& renderer, TextRenderer& textRenderer, const glm::mat4& view, const glm::mat4& projection);
    
    float GetClosestHitDistance() const;

private:
    void CreateNumpadPanel(std::string* numpadInput, std::function<void(SDL_Keycode)> onNumpadKey, bool* isNumpadActive);

    VRManager* vrManager_ = nullptr;
    bool* isNumpadActive_ = nullptr;
    std::map<std::string, std::unique_ptr<VRPanel>> panels_;
    VRUIStyle style_;
    float closestHitDistance_ = -1.0f;
};

}

