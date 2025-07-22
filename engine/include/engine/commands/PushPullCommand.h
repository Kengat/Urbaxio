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
class PushPullCommand : public ICommand {
public:
    PushPullCommand(
        Scene* scene, 
        const std::vector<glm::vec3>& faceVertices,
        const glm::vec3& faceNormal,
        float distance,
        bool disableMerge);

    void Execute() override;
    void Undo() override;
    const char* GetName() const override;

private:
    static std::vector<char> SerializeShape(const TopoDS_Shape& shape);
    void RestoreObjectShape(uint64_t objectId, const std::vector<char>& shapeData);

    Scene* scene_;
    std::vector<glm::vec3> faceVertices_;
    glm::vec3 faceNormal_;
    float distance_;
    bool disableMerge_;
    std::vector<char> shapeBefore_;
    std::vector<char> shapeAfter_;
    uint64_t targetObjectId_ = 0;
    bool isExecuted_ = false;
};

} // namespace Urbaxio::Engine 