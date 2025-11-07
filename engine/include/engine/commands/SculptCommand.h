#pragma once

#include "engine/commands/ICommand.h"

#include <cstdint>
#include <vector>

namespace Urbaxio::Engine {

class Scene; // Forward declaration

class SculptCommand : public ICommand {
public:
    SculptCommand(Scene* scene, uint64_t objectId, 
                  std::vector<float>&& dataBefore, 
                  std::vector<float>&& dataAfter);

    void Execute() override;
    void Undo() override;
    const char* GetName() const override;

private:
    void applyGridData(const std::vector<float>& data);

    Scene* scene_;
    uint64_t objectId_;
    std::vector<float> dataBefore_;
    std::vector<float> dataAfter_;
};

} // namespace Urbaxio::Engine

