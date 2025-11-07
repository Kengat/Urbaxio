#pragma once

#include "engine/commands/ICommand.h"
#include <cstdint>
#include <memory>

namespace Urbaxio::Engine {
    class Scene;
    class IGeometry; // Forward declaration
}


namespace Urbaxio::Engine {


class VoxelizeCommand : public ICommand {
public:
    VoxelizeCommand(Scene* scene, uint64_t objectId, std::unique_ptr<IGeometry> voxelizedGeometry);
    ~VoxelizeCommand();

    void Execute() override;
    void Undo() override;
    const char* GetName() const override;

private:
    Scene* scene_;
    uint64_t objectId_;

    // Memento: store the original geometry to allow undo
    std::unique_ptr<IGeometry> originalGeometry_;
    std::unique_ptr<IGeometry> voxelizedGeometry_;
};


} // namespace Urbaxio::Engine

