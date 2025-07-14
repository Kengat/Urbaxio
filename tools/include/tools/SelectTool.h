#pragma once

#include "tools/ITool.h"
#include <cstdint>
#include <cstddef> // for size_t

namespace Urbaxio::Tools {

class SelectTool : public ITool {
public:
    ToolType GetType() const override { return ToolType::Select; }
    const char* GetName() const override { return "Select"; }

    void Activate(const ToolContext& context) override;
    void Deactivate() override;

    void OnLeftMouseDown(int mouseX, int mouseY, bool shift, bool ctrl) override;

private:
    // Double-click detection state
    uint32_t lastClickTimestamp = 0;
    uint64_t lastClickedObjId = 0;
    size_t lastClickedTriangleIndex = 0;
};

} // namespace Urbaxio::Tools 