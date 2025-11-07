#pragma once

#include "file_io.h"

#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <functional>
#include <filesystem>
#include <variant>

// Forward declarations for our data types

namespace Urbaxio::Engine {
    class Scene;
    class VoxelGrid;
}

namespace Urbaxio {

struct VoxelizeResult {
    uint64_t objectId;
    std::unique_ptr<Engine::VoxelGrid> grid;
};

// A variant to hold any type of loaded data result

using LoadedDataResult = std::variant<FileIO::LoadedSceneData, VoxelizeResult>;

class LoadingManager {
public:
    LoadingManager();
    ~LoadingManager();

    void RequestLoadObj(const std::string& filepath, float scale);

    void RequestVoxelize(Engine::Scene* scene, uint64_t objectId, int resolution);

    bool IsLoading() const;

    float GetProgress() const;

    std::string GetStatus() const;

    bool PopResult(LoadedDataResult& outData);

private:
    void worker_thread_main();

    struct LoadJob {
        std::string filepath;
        float scale;
    };

    struct VoxelizeJob {
        uint64_t objectId;
        int resolution;
        std::vector<char> serializedShape; // Pass shape data safely to worker thread
    };

    std::thread workerThread_;
    std::queue<std::variant<LoadJob, VoxelizeJob>> jobQueue_;
    std::queue<LoadedDataResult> resultQueue_;
    std::mutex jobMutex_;
    std::mutex resultMutex_;
    std::condition_variable jobCondition_;
    bool stopWorker_ = false;
    std::atomic<bool> isLoading_;
    std::atomic<float> progress_;
    std::string statusMessage_;
    mutable std::mutex statusMutex_;
};

} // namespace Urbaxio

