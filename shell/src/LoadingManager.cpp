#include "LoadingManager.h"

#include <iostream>

namespace Urbaxio {

LoadingManager::LoadingManager() {
    isLoading_ = false;
    progress_ = 0.0f;
    stopWorker_ = false;
    workerThread_ = std::thread(&LoadingManager::worker_thread_main, this);
}

LoadingManager::~LoadingManager() {
    {
        std::unique_lock<std::mutex> lock(jobMutex_);
        stopWorker_ = true;
    }
    jobCondition_.notify_one();
    if (workerThread_.joinable()) {
        workerThread_.join();
    }
}

void LoadingManager::RequestLoadObj(const std::string& filepath, float scale) {
    if (isLoading_) {
        std::cout << "LoadingManager: A file is already being loaded." << std::endl;
        return;
    }

    {
        std::unique_lock<std::mutex> lock(jobMutex_);
        jobQueue_.push({filepath, scale});
    }
    jobCondition_.notify_one();
}

bool LoadingManager::IsLoading() const {
    return isLoading_;
}

float LoadingManager::GetProgress() const {
    return progress_;
}

std::string LoadingManager::GetStatus() const {
    std::lock_guard<std::mutex> lock(statusMutex_);
    return statusMessage_;
}

bool LoadingManager::PopResult(Urbaxio::FileIO::LoadedSceneData& outData) {
    std::lock_guard<std::mutex> lock(resultMutex_);
    if (resultQueue_.empty()) {
        return false;
    }

    outData = std::move(resultQueue_.front());
    resultQueue_.pop();
    return true;
}

void LoadingManager::worker_thread_main() {
    while (true) {
        LoadJob job;

        {
            std::unique_lock<std::mutex> lock(jobMutex_);
            jobCondition_.wait(lock, [this] { return !jobQueue_.empty() || stopWorker_; });

            if (stopWorker_) return;

            job = std::move(jobQueue_.front());
            jobQueue_.pop();
        }

        isLoading_ = true;
        progress_ = 0.0f;

        {
            std::lock_guard<std::mutex> lock(statusMutex_);
            statusMessage_ = "Parsing " + std::filesystem::path(job.filepath).filename().string() + "...";
        }

        Urbaxio::FileIO::LoadedSceneData result = Urbaxio::FileIO::LoadObjToIntermediate(job.filepath, job.scale, progress_);

        {
            std::lock_guard<std::mutex> lock(resultMutex_);
            resultQueue_.push(std::move(result));
        }

        isLoading_ = false;
    }
}

} // namespace Urbaxio

