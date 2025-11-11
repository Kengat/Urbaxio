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
#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/Types.h>
#include <cstring>

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
                isBlockingOperation_ = false; // Remeshing is a background task
                { std::lock_guard<std::mutex> lock(statusMutex_); statusMessage_ = "Remeshing object " + std::to_string(job.objectId) + "..."; }
                
                CadKernel::MeshBuffers outMesh;
                openvdb::FloatGrid::Ptr grid = job.gridCopy->grid_;
                
                if (grid) {
                    // --- START OF RE-ARCHITECTURE: Using openvdb::tools::volumeToMesh ---
                    
                    std::vector<openvdb::Vec3s> points;
                    std::vector<openvdb::Vec3I> triangles;
                    std::vector<openvdb::Vec4I> quads;
                    const double isoValue = 0.0;
                    const double adaptivity = 0.0; // 0.0 for uniform meshing (like Marching Cubes), > 0 for adaptive (Dual Contouring)
                    try {
                        // This function works directly on the sparse grid and is multi-threaded with TBB.
                        openvdb::tools::volumeToMesh(*grid, points, triangles, quads, isoValue, adaptivity);
                        if (!points.empty() && (!triangles.empty() || !quads.empty())) {
                            // 1. Convert points
                            outMesh.vertices.reserve(points.size() * 3);
                            for (const openvdb::Vec3s& p : points) {
                                outMesh.vertices.push_back(p.x());
                                outMesh.vertices.push_back(p.y());
                                outMesh.vertices.push_back(p.z());
                            }
                            // 2. Convert triangles and triangulate quads, FIXING winding order
                            outMesh.indices.reserve(triangles.size() * 3 + quads.size() * 6);
                            for (const openvdb::Vec3I& t : triangles) {
                                outMesh.indices.push_back(t.x());
                                outMesh.indices.push_back(t.z()); // Swapped
                                outMesh.indices.push_back(t.y()); // Swapped
                            }
                            for (const openvdb::Vec4I& q : quads) {
                                // Triangulate quad [x, y, z, w] into two triangles [x, z, y] and [x, w, z] to flip normals
                                outMesh.indices.push_back(q.x());
                                outMesh.indices.push_back(q.z()); // Swapped
                                outMesh.indices.push_back(q.y()); // Swapped
                                
                                outMesh.indices.push_back(q.x());
                                outMesh.indices.push_back(q.w()); // Swapped
                                outMesh.indices.push_back(q.z()); // Swapped
                            }
                            // 3. Calculate normals (using existing igl code path, as it's fast)
                            Eigen::MatrixXd V_igl(points.size(), 3);
                            for(size_t i = 0; i < points.size(); ++i) {
                                V_igl(i, 0) = points[i].x();
                                V_igl(i, 1) = points[i].y();
                                V_igl(i, 2) = points[i].z();
                            }
                            
                            Eigen::MatrixXi F_igl(outMesh.indices.size() / 3, 3);
                            for (size_t i = 0; i < F_igl.rows(); ++i) {
                                F_igl(i, 0) = outMesh.indices[i * 3 + 0];
                                F_igl(i, 1) = outMesh.indices[i * 3 + 1];
                                F_igl(i, 2) = outMesh.indices[i * 3 + 2];
                            }
                            Eigen::MatrixXd N_igl;
                            igl::per_vertex_normals(V_igl, F_igl, N_igl);
                            
                            outMesh.normals.resize(N_igl.rows() * 3);
                            for (int i = 0; i < N_igl.rows(); ++i) {
                                outMesh.normals[i * 3 + 0] = (float)N_igl(i, 0);
                                outMesh.normals[i * 3 + 1] = (float)N_igl(i, 1);
                                outMesh.normals[i * 3 + 2] = (float)N_igl(i, 2);
                            }
                            
                            // 4. Generate placeholder UVs
                            outMesh.uvs.assign(points.size() * 2, 0.0f);
                            
                            std::cout << "[LoadingManager] Remesh complete (OpenVDB): " << points.size() 
                                      << " vertices, " << outMesh.indices.size() / 3 << " triangles" << std::endl;
                        }
                    } catch (const openvdb::Exception& e) {
                        std::cerr << "[LoadingManager] OpenVDB volumeToMesh failed: " << e.what() << std::endl;
                    }
                    // --- END OF RE-ARCHITECTURE ---
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

