#pragma once

#include "engine/commands/ICommand.h"

#include <cstdint>
#include <vector>
#include <openvdb/openvdb.h>

namespace Urbaxio::Engine {

class Scene; // Forward declaration

class SculptCommand : public ICommand {
public:
    // MODIFIED: Constructor now accepts bounding boxes for before/after states
    SculptCommand(Scene* scene, uint64_t objectId,
                  std::vector<float>&& dataBefore,
                  std::vector<float>&& dataAfter,
                  openvdb::CoordBBox beforeBBox,
                  openvdb::CoordBBox afterBBox);

    void Execute() override;
    void Undo() override;
    const char* GetName() const override;

private:
    // MODIFIED: applyGridData now requires a bounding box to restore correctly
    void applyGridData(const std::vector<float>& data, const openvdb::CoordBBox& bbox);

    Scene* scene_;
    uint64_t objectId_;
    std::vector<float> dataBefore_;
    std::vector<float> dataAfter_;

    // NEW: Store bounding boxes for Undo/Redo
    openvdb::CoordBBox beforeBBox_;
    openvdb::CoordBBox afterBBox_;
};

} // namespace Urbaxio::Engine

