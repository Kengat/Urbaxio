#pragma once

#include "tools/ITool.h"
#include <cad_kernel/MeshBuffers.h>
#include <glm/glm.hpp>
#include <cstdint>
#include <cstddef>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <map>
#include "engine/line.h"

namespace Urbaxio::Tools {

struct SelectionJob {
    glm::vec3 box_min;
    glm::vec3 box_max;
    bool isWindowSelection;
    bool shift;
    std::map<uint64_t, CadKernel::MeshBuffers> objectMeshes;
    std::map<uint64_t, std::set<uint64_t>> objectLineIDs;
    std::map<uint64_t, Engine::Line> allLines;
};

struct SelectionResult {
    bool shift;
    uint64_t objectId = 0;
    std::vector<size_t> triangleIndices;
    std::set<uint64_t> lineIDs;
};

class SelectTool : public ITool {
public:
    ToolType GetType() const override { return ToolType::Select; }
    const char* GetName() const override { return "Select"; }

    SelectTool();
    ~SelectTool();

    void Activate(const ToolContext& context) override;
    void Deactivate() override;

    void OnLeftMouseDown(int mouseX, int mouseY, bool shift, bool ctrl, const glm::vec3& rayOrigin = {}, const glm::vec3& rayDirection = {}) override;
    void OnLeftMouseUp(int mouseX, int mouseY, bool shift, bool ctrl) override;
    void OnMouseMove(int mouseX, int mouseY) override;
    void OnUpdate(const SnapResult& snap, const glm::vec3& rayOrigin = {}, const glm::vec3& rayDirection = {}) override;

    // NEW: Method specifically for finalizing a VR drag, using the correct view matrix
    void FinalizeVrDragSelection(const glm::mat4& centerEyeViewMatrix, bool shift);
    void RenderPreview(Renderer& renderer, const SnapResult& snap) override;

    // --- Public methods to query drag state ---
    bool IsDragging() const;
    void GetDragRect(glm::vec2& outStart, glm::vec2& outCurrent) const;
    
    // NEW: Public methods for VR 3D drag box
    bool IsVrDragging() const;
    void GetVrDragBoxCorners(glm::vec3& outStart, glm::vec3& outEnd) const;
    float GetVrDragDistanceOffset() const;
    void GetVrGhostPoint(const glm::vec3& rayOrigin, const glm::vec3& rayDirection, glm::vec3& outPoint) const;
    float GetVrGhostPointAlpha() const;


private:
    // Double-click detection state
    uint32_t lastClickTimestamp = 0;
    uint64_t lastClickedObjId = 0;
    size_t lastClickedTriangleIndex = 0;

    // Desktop drag selection state
    bool isMouseDown = false;
    bool isDragging = false;
    glm::vec2 dragStartCoords;
    glm::vec2 currentDragCoords;

    // VR interaction state
    bool isVrTriggerDown = false;
    bool isVrDragging = false;
    uint32_t vrTriggerDownTimestamp = 0;
    SnapResult vrClickSnapResult; // Stores the snap result at the moment of the click
    glm::vec3 vrDragStartPoint;
    glm::vec3 vrDragEndPoint;
    float vrDragDistanceOffset = 0.0f;
    float vrGhostPointAlpha_ = 0.0f;

    std::thread workerThread_;
    std::queue<SelectionJob> jobQueue_;
    std::queue<SelectionResult> resultQueue_;
    std::mutex jobMutex_;
    std::mutex resultMutex_;
    std::condition_variable jobCondition_;
    bool stopWorker_ = false;

    void worker_thread_main();
};

} // namespace Urbaxio::Tools 