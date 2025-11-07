#include "LoadingManager.h"
#include "engine/scene.h"
#include "engine/scene_object.h"
#include "file_io.h"
#include "engine/geometry/BRepGeometry.h"
#include "engine/geometry/VolumetricGeometry.h"
#include "engine/geometry/VoxelGrid.h"
#include "Voxelizer.h"
#include <TopoDS_Shape.hxx>
#include <BinTools.hxx>
#include <sstream>

// For Marching Cubes
#include <igl/marching_cubes.h>
#include <igl/per_vertex_normals.h>
#include <Eigen/Core>

#include <iostream>

namespace Urbaxio {

LoadingManager::LoadingManager() {
    isLoading_ = false;
    isBlockingOperation_ = false;
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

void LoadingManager::RequestRemesh(Engine::Scene* scene, uint64_t objectId) {
    // DON'T reject if busy - queue the remesh request!
    // This prevents objects from disappearing
    
    Engine::SceneObject* obj = scene->get_object_by_id(objectId);
    auto* volGeom = obj ? dynamic_cast<Engine::VolumetricGeometry*>(obj->getGeometry()) : nullptr;
    if (!volGeom || !volGeom->getGrid()) {
        std::cout << "LoadingManager: Object is not volumetric. Remesh skipped." << std::endl;
        return;
    }

    // Copy the grid data to pass safely to worker thread
    const Engine::VoxelGrid* original = volGeom->getGrid();
    auto gridCopy = std::make_unique<Engine::VoxelGrid>(
        original->dimensions, 
        original->origin, 
        original->voxelSize
    );
    
    // Deep copy the OpenVDB grid (cannot use operator=, must use deepCopy)
    gridCopy->grid_ = original->grid_->deepCopy();
    
    RemeshJob job;
    job.objectId = objectId;
    job.gridCopy = std::move(gridCopy);

    {
        std::unique_lock<std::mutex> lock(jobMutex_);
        jobQueue_.push(std::move(job));
    }
    jobCondition_.notify_one();
    
    std::cout << "LoadingManager: Remesh request queued for object " << objectId 
              << (isLoading_ ? " (worker busy, will process after current job)" : "") << std::endl;
}

bool LoadingManager::IsLoading() const {
    return isLoading_;
}

bool LoadingManager::IsBlockingOperation() const {
    return isBlockingOperation_;
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
        std::variant<LoadJob, VoxelizeJob, RemeshJob> job_variant;

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
                isBlockingOperation_ = true; // BLOCKS UI
                { std::lock_guard<std::mutex> lock(statusMutex_); statusMessage_ = "Parsing " + std::filesystem::path(job.filepath).filename().string() + "..."; }
                FileIO::LoadedSceneData result = FileIO::LoadObjToIntermediate(job.filepath, job.scale, progress_);
                { std::lock_guard<std::mutex> lock(resultMutex_); resultQueue_.push(std::move(result)); }
            } else if constexpr (std::is_same_v<T, VoxelizeJob>) {
                isBlockingOperation_ = true; // BLOCKS UI
                { std::lock_guard<std::mutex> lock(statusMutex_); statusMessage_ = "Voxelizing object " + std::to_string(job.objectId) + "..."; }
                TopoDS_Shape shape;
                std::stringstream ss(std::string(job.serializedShape.begin(), job.serializedShape.end()));
                BinTools::Read(shape, ss);
                auto grid = Voxelizer::BRepToSDFGrid(shape, job.resolution);
                VoxelizeResult result{job.objectId, std::move(grid)};
                { std::lock_guard<std::mutex> lock(resultMutex_); resultQueue_.push(std::move(result)); }
            } else if constexpr (std::is_same_v<T, RemeshJob>) {
                isBlockingOperation_ = false; // DOES NOT BLOCK UI (silent background operation)
                { std::lock_guard<std::mutex> lock(statusMutex_); statusMessage_ = "Remeshing object " + std::to_string(job.objectId) + "..."; }
                
                // Run Marching Cubes on the copied grid (THIS IS THE EXPENSIVE OPERATION)
                CadKernel::MeshBuffers outMesh;
                Engine::VoxelGrid* grid = job.gridCopy.get();
                
                if (grid) {
                    const auto& dims = grid->dimensions;
                    const size_t totalVoxels = dims.x * dims.y * dims.z;
                    
                    // Convert sparse OpenVDB grid to dense array for Marching Cubes
                    std::vector<float> denseData = grid->toDenseArray();
                    
                    // Prepare data for libigl
                    Eigen::VectorXd S(totalVoxels);
                    Eigen::MatrixXd GV(totalVoxels, 3);
                    
                    for (unsigned int z = 0; z < dims.z; ++z) {
                        for (unsigned int y = 0; y < dims.y; ++y) {
                            for (unsigned int x = 0; x < dims.x; ++x) {
                                size_t index = z * dims.x * dims.y + y * dims.x + x;
                                S(index) = denseData[index];
                                
                                GV.row(index) << 
                                    grid->origin.x + x * grid->voxelSize,
                                    grid->origin.y + y * grid->voxelSize,
                                    grid->origin.z + z * grid->voxelSize;
                            }
                        }
                    }
                    
                    // Run Marching Cubes
                    Eigen::MatrixXd V_igl;
                    Eigen::MatrixXi F_igl;
                    
                    try {
                        igl::marching_cubes(S, GV, (int)dims.x, (int)dims.y, (int)dims.z, 0.0, V_igl, F_igl);
                        
                        if (V_igl.rows() > 0 && F_igl.rows() > 0) {
                            // Convert to MeshBuffers
                            outMesh.vertices.resize(V_igl.rows() * 3);
                            for (int i = 0; i < V_igl.rows(); ++i) {
                                outMesh.vertices[i * 3 + 0] = (float)V_igl(i, 0);
                                outMesh.vertices[i * 3 + 1] = (float)V_igl(i, 1);
                                outMesh.vertices[i * 3 + 2] = (float)V_igl(i, 2);
                            }
                            
                            outMesh.indices.resize(F_igl.rows() * 3);
                            for (int i = 0; i < F_igl.rows(); ++i) {
                                outMesh.indices[i * 3 + 0] = (unsigned int)F_igl(i, 0);
                                outMesh.indices[i * 3 + 1] = (unsigned int)F_igl(i, 1);
                                outMesh.indices[i * 3 + 2] = (unsigned int)F_igl(i, 2);
                            }
                            
                            // Calculate normals
                            Eigen::MatrixXd N_igl;
                            igl::per_vertex_normals(V_igl, F_igl, N_igl);
                            
                            outMesh.normals.resize(N_igl.rows() * 3);
                            for (int i = 0; i < N_igl.rows(); ++i) {
                                outMesh.normals[i * 3 + 0] = (float)N_igl(i, 0);
                                outMesh.normals[i * 3 + 1] = (float)N_igl(i, 1);
                                outMesh.normals[i * 3 + 2] = (float)N_igl(i, 2);
                            }
                            
                            outMesh.uvs.assign(V_igl.rows() * 2, 0.0f);
                            
                            std::cout << "[LoadingManager] Remesh complete: " << V_igl.rows() 
                                      << " vertices, " << F_igl.rows() << " triangles" << std::endl;
                        }
                    } catch (const std::exception& e) {
                        std::cerr << "[LoadingManager] Marching Cubes failed: " << e.what() << std::endl;
                    }
                }
                
                RemeshResult result{job.objectId, std::move(outMesh)};
                { std::lock_guard<std::mutex> lock(resultMutex_); resultQueue_.push(std::move(result)); }
            }
        }, job_variant);

        isLoading_ = false;
        isBlockingOperation_ = false;
    }
}

} // namespace Urbaxio

