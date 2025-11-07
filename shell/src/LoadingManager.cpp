#include "LoadingManager.h"
#include "engine/scene.h"
#include "engine/scene_object.h"
#include "file_io.h"
#include "engine/geometry/BRepGeometry.h"
#include "engine/geometry/VoxelGrid.h"
#include "Voxelizer.h"
#include <TopoDS_Shape.hxx>
#include <BinTools.hxx>
#include <sstream>

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
        jobQueue_.push(LoadJob{filepath, scale});
    }
    jobCondition_.notify_one();
}

void LoadingManager::RequestVoxelize(Engine::Scene* scene, uint64_t objectId, int resolution) {
    if (isLoading_) {
        std::cout << "LoadingManager: An operation is already in progress." << std::endl;
        return;
    }
    Engine::SceneObject* obj = scene->get_object_by_id(objectId);
    auto* brepGeom = obj ? dynamic_cast<Engine::BRepGeometry*>(obj->getGeometry()) : nullptr;
    if (!brepGeom || !brepGeom->getShape()) return;

    VoxelizeJob job;
    job.objectId = objectId;
    job.resolution = resolution;
    // Serialize shape to pass it safely to the worker thread
    std::stringstream ss;
    BinTools::Write(*brepGeom->getShape(), ss);
    std::string const& s = ss.str();
    job.serializedShape = std::vector<char>(s.begin(), s.end());

    {
        std::unique_lock<std::mutex> lock(jobMutex_);
        jobQueue_.push(job);
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

bool LoadingManager::PopResult(LoadedDataResult& outData) {
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
        std::variant<LoadJob, VoxelizeJob> job_variant;

        {
            std::unique_lock<std::mutex> lock(jobMutex_);
            jobCondition_.wait(lock, [this] { return !jobQueue_.empty() || stopWorker_; });

            if (stopWorker_) return;

            job_variant = std::move(jobQueue_.front());
            jobQueue_.pop();
        }

        isLoading_ = true;
        progress_ = 0.0f;

        std::visit([this](auto&& job) {
            using T = std::decay_t<decltype(job)>;
            if constexpr (std::is_same_v<T, LoadJob>) {
                { std::lock_guard<std::mutex> lock(statusMutex_); statusMessage_ = "Parsing " + std::filesystem::path(job.filepath).filename().string() + "..."; }
                FileIO::LoadedSceneData result = FileIO::LoadObjToIntermediate(job.filepath, job.scale, progress_);
                { std::lock_guard<std::mutex> lock(resultMutex_); resultQueue_.push(std::move(result)); }
            } else if constexpr (std::is_same_v<T, VoxelizeJob>) {
                { std::lock_guard<std::mutex> lock(statusMutex_); statusMessage_ = "Voxelizing object " + std::to_string(job.objectId) + "..."; }
                TopoDS_Shape shape;
                std::stringstream ss(std::string(job.serializedShape.begin(), job.serializedShape.end()));
                BinTools::Read(shape, ss);
                auto grid = Voxelizer::BRepToSDFGrid(shape, job.resolution);
                VoxelizeResult result{job.objectId, std::move(grid)};
                { std::lock_guard<std::mutex> lock(resultMutex_); resultQueue_.push(std::move(result)); }
            }
        }, job_variant);

        isLoading_ = false;
    }
}

} // namespace Urbaxio

