#pragma once

#include "engine/commands/ICommand.h"
#include <glm/glm.hpp>
#include <vector>
#include <cstdint>
#include <memory> // For std::unique_ptr

namespace Urbaxio::Engine {
    class Scene;
    struct SceneState; // Forward declaration for our Memento is sufficient here
}

namespace Urbaxio::Engine {

// Command to add a user-defined line.
// This is a complex command as it can modify the entire scene topology.
// It uses the Memento pattern on the whole scene.
class CreateLineCommand : public ICommand {
public:
    CreateLineCommand(Scene* scene, const glm::vec3& start, const glm::vec3& end);
    // The destructor MUST be declared here but defined in the .cpp file
    // to handle the std::unique_ptr to an incomplete type (PIMPL idiom).
    ~CreateLineCommand();

    void Execute() override;
    void Undo() override;
    const char* GetName() const override;

private:
    Scene* scene_;
    glm::vec3 start_;
    glm::vec3 end_;
    
    // Memento: pointers to scene state before and after.
    std::unique_ptr<SceneState> stateBefore_;
    std::unique_ptr<SceneState> stateAfter_;
    bool isExecuted_ = false;
};

} // namespace Urbaxio::Engine 