#include "engine/commands/MoveSubObjectCommand.h"
#include "engine/scene.h"

namespace Urbaxio::Engine {

MoveSubObjectCommand::MoveSubObjectCommand(
    Scene* scene,
    uint64_t objectId,
    SubObjectType type,
    const std::vector<glm::vec3>& initialPositions,
    const glm::vec3& translationVector)
    : scene_(scene),
      objectId_(objectId),
      type_(type),
      initialPositions_(initialPositions),
      translationVector_(translationVector) {
}

MoveSubObjectCommand::~MoveSubObjectCommand() = default;

const char* MoveSubObjectCommand::GetName() const {
    return "Move Sub-Object";
}

void MoveSubObjectCommand::Execute() {
    if (isExecuted_ && stateAfter_) {
        // This is a Redo operation
        scene_->RestoreState(*stateAfter_);
        return;
    }

    // This is the first execution
    stateBefore_ = scene_->CaptureState();

    // Передаем тип в метод реконструкции
    scene_->RebuildObjectByMovingVertices(objectId_, type_, initialPositions_, translationVector_);

    stateAfter_ = scene_->CaptureState();
    isExecuted_ = true;
}

void MoveSubObjectCommand::Undo() {
    if (stateBefore_) {
        scene_->RestoreState(*stateBefore_);
    }
}

} // namespace Urbaxio::Engine 