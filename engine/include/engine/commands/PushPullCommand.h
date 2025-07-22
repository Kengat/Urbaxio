#pragma once

#include "engine/commands/ICommand.h"
#include <glm/glm.hpp>
#include <vector>
#include <cstdint>

// Forward declarations
class TopoDS_Shape;

namespace Urbaxio::Engine {
    class Scene;
}

namespace Urbaxio::Engine {

// A command to perform a push/pull (extrusion) operation on a face.
// This command uses the Memento pattern to store the object's state
// before and after the operation, ensuring a reliable undo/redo.
class PushPullCommand : public ICommand {
public:
    PushPullCommand(
        Scene* scene, 
        uint64_t objectId, 
        const std::vector<size_t>& faceIndices, 
        const glm::vec3& direction, 
        float distance,
        bool disableMerge);

    void Execute() override;
    void Undo() override;
    const char* GetName() const override;

private:
    // Helper to serialize a shape to a byte vector (creates a Memento).
    static std::vector<char> SerializeShape(const TopoDS_Shape& shape);
    // Helper to apply a Memento to an object (restores state).
    void RestoreObjectShape(const std::vector<char>& shapeData);

    Scene* scene_;
    uint64_t objectId_;
    std::vector<size_t> faceIndices_;
    glm::vec3 direction_;
    float distance_;
    bool disableMerge_;

    // Memento: serialized shapes as byte vectors.
    std::vector<char> shapeBefore_;
    std::vector<char> shapeAfter_;
    bool isExecuted_ = false;
};

} // namespace Urbaxio::Engine 