#pragma once

#include "engine/commands/ICommand.h"
#include <glm/glm.hpp>
#include <vector>
#include <cstdint>
#include <memory>

namespace Urbaxio::Engine {
    class Scene;
    struct SceneState; // Memento

    // Enum был перемещен в ICommand.h
}

namespace Urbaxio::Engine {

// Command to move sub-objects (vertices, edges, faces) of a B-Rep shape.
// This is a complex transactional command using the Memento pattern on the scene.
class MoveSubObjectCommand : public ICommand {
public:
    MoveSubObjectCommand(
        Scene* scene,
        uint64_t objectId,
        SubObjectType type,
        const std::vector<glm::vec3>& initialPositions, // Original positions of vertices to move
        const glm::vec3& translationVector
    );
    ~MoveSubObjectCommand();

    void Execute() override;
    void Undo() override;
    const char* GetName() const override;

private:
    Scene* scene_;
    uint64_t objectId_;
    SubObjectType type_;
    std::vector<glm::vec3> initialPositions_;
    glm::vec3 translationVector_;

    // Memento: pointers to scene state before and after.
    std::unique_ptr<SceneState> stateBefore_;
    std::unique_ptr<SceneState> stateAfter_;
    bool isExecuted_ = false;
};

} // namespace Urbaxio::Engine 