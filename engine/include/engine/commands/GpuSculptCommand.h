#pragma once

#include "engine/commands/ICommand.h"
#include <cstdint>

namespace Urbaxio::Engine {

class Scene;
class GpuVoxelManager;

// GPU-native Undo/Redo command - operates on GPU handles, no CPU copies!
class GpuSculptCommand : public ICommand {
public:
    GpuSculptCommand(
        Scene* scene,
        GpuVoxelManager* gpuManager,
        uint64_t objectId,
        uint64_t* activeHandlePtr,  // Pointer to the active grid handle (will be modified!)
        uint64_t undoHandle,
        uint64_t redoHandle
    );
    
    ~GpuSculptCommand() override;

    void Execute() override;  // Redo: swap to redo handle
    void Undo() override;     // Undo: swap to undo handle
    const char* GetName() const override;

private:
    void swapToHandle(uint64_t targetHandle);
    void remeshObject();

    Scene* scene_;
    GpuVoxelManager* gpuManager_;
    uint64_t objectId_;
    uint64_t* activeHandlePtr_;  // Points to GpuSculptTool::impl_->currentGpuHandle
    
    uint64_t undoHandle_;
    uint64_t redoHandle_;
};

} // namespace Urbaxio::Engine


