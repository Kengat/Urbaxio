#define GLM_ENABLE_EXPERIMENTAL // <-- FIX: Enable experimental GLM features
#include "tools/MoveTool.h"
#include "engine/scene.h"
#include "engine/commands/MoveCommand.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp> // <-- This header requires the define above
#include <iostream>
#include <memory>

namespace Urbaxio::Tools {

void MoveTool::Activate(const ToolContext& context) {
    ITool::Activate(context);
    reset();
}

void MoveTool::Deactivate() {
    reset();
    ITool::Deactivate();
}

void MoveTool::reset() {
    currentState = State::IDLE;
    movingObjectId = 0;
    currentTranslation = glm::vec3(0.0f);
}

void MoveTool::OnLeftMouseDown(int mouseX, int mouseY, bool shift, bool ctrl) {
    if (currentState == State::IDLE) {
        if (*context.selectedObjId != 0) {
            movingObjectId = *context.selectedObjId;
            basePoint = lastSnapResult.worldPoint;
            currentState = State::AWAITING_DESTINATION;
            std::cout << "MoveTool: Started moving object " << movingObjectId << " from base point." << std::endl;
        } else {
            std::cout << "MoveTool: Select an object before using the Move tool." << std::endl;
        }
    } else if (currentState == State::AWAITING_DESTINATION) {
        finalizeMove();
    }
}

void MoveTool::OnRightMouseDown() {
    if (currentState == State::AWAITING_DESTINATION) {
        reset();
    }
}

void MoveTool::OnKeyDown(SDL_Keycode key, bool shift, bool ctrl) {
    if (key == SDLK_ESCAPE) {
        reset();
    }
}

void MoveTool::OnUpdate(const SnapResult& snap) {
    lastSnapResult = snap;
    if (currentState == State::AWAITING_DESTINATION) {
        currentTranslation = snap.worldPoint - basePoint;
    }
}

void MoveTool::finalizeMove() {
    if (movingObjectId != 0 && glm::length2(currentTranslation) > 1e-8f) {
        auto command = std::make_unique<Engine::MoveCommand>(
            context.scene,
            movingObjectId,
            currentTranslation
        );
        context.scene->getCommandManager()->ExecuteCommand(std::move(command));
    }
    reset();
}

bool MoveTool::IsMoving() const {
    return currentState == State::AWAITING_DESTINATION;
}

uint64_t MoveTool::GetMovingObjectId() const {
    return movingObjectId;
}

glm::mat4 MoveTool::GetPreviewTransform() const {
    return glm::translate(glm::mat4(1.0f), currentTranslation);
}

} // namespace Urbaxio::Tools