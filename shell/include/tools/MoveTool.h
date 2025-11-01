#pragma once

#include "tools/ITool.h"
#include <cad_kernel/MeshBuffers.h> // For ghost mesh
#include <glm/mat4x4.hpp>
#include <set> // For std::set

namespace Urbaxio::Tools {

// State of the entire tool
enum class MoveToolState {
    IDLE,             // Waiting for a selection or a click to determine the target
    MOVING_PREVIEW,   // Target is selected, dragging to determine translation
};

// State of the axis locking during movement
enum class AxisLockState {
    FREE,
    AXIS_LOCKED,
    INFERENCE_LOCKED
};

// What is being moved
struct MoveTarget {
    enum class TargetType { NONE, OBJECT, FACE, EDGE, VERTEX };
    TargetType type = TargetType::NONE;
    uint64_t objectId = 0;
    
    // Original vertex indices in the object's mesh for deformation.
    // If type is OBJECT, this contains all vertex indices.
    std::set<unsigned int> movingVertices;
};

class MoveTool : public ITool {
public:
    ToolType GetType() const override { return ToolType::Move; }
    const char* GetName() const override { return "Move"; }

    void Activate(const ToolContext& context) override;
    void Deactivate() override;

    void OnLeftMouseDown(int mouseX, int mouseY, bool shift, bool ctrl, const glm::vec3& rayOrigin = {}, const glm::vec3& rayDirection = {}) override;
    void OnRightMouseDown() override;
    void OnUpdate(const SnapResult& snap, const glm::vec3& rayOrigin = {}, const glm::vec3& rayDirection = {}) override;
    void OnKeyDown(SDL_Keycode key, bool shift, bool ctrl) override;
    void RenderUI() override;
    
    // Public getters for the main loop to query state
    bool IsMoving() const;
    uint64_t GetMovingObjectId() const; // ID of the object to hide
    const CadKernel::MeshBuffers* GetGhostMesh() const;
    const std::vector<unsigned int>* GetGhostWireframeIndices() const; // <-- ДОБАВИТЬ ЭТОТ МЕТОД

private:
    void reset();
    void startMove(const SnapResult& snap);
    void finalizeMove();

    // High-level state
    MoveToolState currentState = MoveToolState::IDLE;
    MoveTarget currentTarget;

    // Movement state
    glm::dvec3 basePoint{0.0};
    glm::dvec3 currentTranslation{0.0};
    
    // Axis Locking State
    AxisLockState lockState = AxisLockState::FREE;
    glm::dvec3 lockedAxisDir{0.0};
    glm::dvec3 inferenceAxisDir{0.0};

    // Ghost mesh for preview
    CadKernel::MeshBuffers ghostMesh;
    std::vector<unsigned int> ghostWireframeIndices; // <-- ИЗМЕНИТЬ ЭТО
    bool ghostMeshActive = false;

    // Input buffer for length
    char lengthInputBuf[64] = "";

    // Helpers
    void determineTargetFromSelection();
    void determineTargetFromPick(int mouseX, int mouseY);
    void updateGhostMeshDeformation();
    bool tryToLockAxis(const glm::dvec3& currentTarget);
    glm::dvec3 calculateAxisLockedPoint(const SnapResult& snap);
    glm::dvec3 calculateInferenceLockedPoint(const SnapResult& snap);
};

} // namespace Urbaxio::Tools