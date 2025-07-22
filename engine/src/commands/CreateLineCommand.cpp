#include "engine/commands/CreateLineCommand.h"
#include "engine/scene.h" // This include brings in the full definition of SceneState

namespace Urbaxio::Engine {

CreateLineCommand::CreateLineCommand(Scene* scene, const glm::vec3& start, const glm::vec3& end)
    : scene_(scene), start_(start), end_(end) {
}

// Define the destructor here in the .cpp file, where SceneState is a complete type.
CreateLineCommand::~CreateLineCommand() = default;

const char* CreateLineCommand::GetName() const {
    return "Create Line";
}

void CreateLineCommand::Execute() {
    if (isExecuted_ && stateAfter_) {
        scene_->RestoreState(*stateAfter_);
        return;
    }

    stateBefore_ = scene_->CaptureState();
    scene_->AddUserLine(start_, end_);
    stateAfter_ = scene_->CaptureState();
    
    isExecuted_ = true;
}

void CreateLineCommand::Undo() {
    if (stateBefore_) {
        scene_->RestoreState(*stateBefore_);
    }
}

} // namespace Urbaxio::Engine 