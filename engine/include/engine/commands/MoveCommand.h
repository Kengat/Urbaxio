#pragma once

#include "engine/commands/ICommand.h"
#include <glm/glm.hpp>
#include <cstdint>

// Forward declarations
namespace Urbaxio::Engine {
    class Scene;
}

namespace Urbaxio::Engine {

// Command to move a scene object by a given vector.
class MoveCommand : public ICommand {
public:
    MoveCommand(Scene* scene, uint64_t objectId, const glm::vec3& translationVector);

    void Execute() override;
    void Undo() override;
    const char* GetName() const override;

private:
    void applyTransform(const glm::vec3& vector);

    Scene* scene_;
    uint64_t objectId_;
    glm::vec3 translationVector_;
};

} // namespace Urbaxio::Engine