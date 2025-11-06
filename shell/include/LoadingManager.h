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

namespace Urbaxio {

class LoadingManager {
public:
    LoadingManager();
    ~LoadingManager();

    void RequestLoadObj(const std::string& filepath, float scale);

    bool IsLoading() const;

    float GetProgress() const;

    std::string GetStatus() const;

    bool PopResult(Urbaxio::FileIO::LoadedSceneData& outData);

private:
    void worker_thread_main();

    struct LoadJob {
        std::string filepath;
        float scale;
    };

    std::thread workerThread_;
    std::queue<LoadJob> jobQueue_;
    std::queue<Urbaxio::FileIO::LoadedSceneData> resultQueue_;
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

