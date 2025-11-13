#pragma once

#include "engine/GpuMesh.h"
#include <cad_kernel/MeshBuffers.h> // For the mesh data source
#include <string>
#include <map>
#include <memory>

namespace Urbaxio::Engine {

    // Manages the lifecycle of mesh data on the GPU.
    // This class ensures that identical meshes are only uploaded once.
    class MeshManager {
    public:
        ~MeshManager();

        // Adds a mesh from CPU buffers, uploads it to the GPU, and returns its ID.
        // If a mesh with this ID already exists, it will be overwritten.
        GpuMesh* AddMesh(const std::string& meshId, const CadKernel::MeshBuffers& meshData);

        // Updates mesh directly from GPU device pointers (Device-to-Device, no CPU copy!)
        GpuMesh* UpdateMeshFromGpuBuffers(
            const std::string& meshId,
            float* d_vertices,
            float* d_normals,
            size_t vertexCount,
            void* cudaStream = nullptr
        );

        // Retrieves a pointer to a GpuMesh by its ID. Returns nullptr if not found.
        GpuMesh* GetMesh(const std::string& meshId);
        const GpuMesh* GetMesh(const std::string& meshId) const;

        // Deletes a specific mesh from both the manager and the GPU.
        void DeleteMesh(const std::string& meshId);

        // Clears all meshes and releases all GPU resources.
        void Clear();

    private:
        void freeGpuResources(GpuMesh& mesh);
        std::map<std::string, GpuMesh> meshes_;
    };

} // namespace Urbaxio::Engine


