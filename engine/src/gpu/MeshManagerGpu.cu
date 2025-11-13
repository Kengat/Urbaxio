#include "engine/MeshManager.h"
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include <glm/glm.hpp>
#include <iostream>
#include <vector>

namespace Urbaxio::Engine {

GpuMesh* MeshManager::UpdateMeshFromGpuBuffers(
    const std::string& meshId,
    float* d_vertices,
    float* d_normals,
    size_t vertexCount,
    void* cudaStream)
{
    if (!d_vertices || !d_normals || vertexCount == 0) {
        std::cerr << "[MeshManager] Invalid GPU buffer pointers!" << std::endl;
        return nullptr;
    }

    auto it = meshes_.find(meshId);
    if (it == meshes_.end()) {
        std::cout << "[MeshManager] Creating new mesh '" << meshId << "'" << std::endl;

        GpuMesh newMesh;
        glGenVertexArrays(1, &newMesh.vao);
        glBindVertexArray(newMesh.vao);

        glGenBuffers(1, &newMesh.vbo_vertices);
        glGenBuffers(1, &newMesh.vbo_normals);
        glGenBuffers(1, &newMesh.ebo);
        glGenBuffers(1, &newMesh.vbo_instances);

        glBindBuffer(GL_ARRAY_BUFFER, newMesh.vbo_instances);
        for (int i = 0; i < 4; ++i) {
            GLuint attribLocation = 3 + i;
            glEnableVertexAttribArray(attribLocation);
            glVertexAttribPointer(
                attribLocation,
                4,
                GL_FLOAT,
                GL_FALSE,
                sizeof(glm::mat4),
                reinterpret_cast<void*>(sizeof(glm::vec4) * i));
            glVertexAttribDivisor(attribLocation, 1);
        }

        glBindVertexArray(0);

        meshes_[meshId] = newMesh;
        it = meshes_.find(meshId);
    }

    GpuMesh& mesh = it->second;
    cudaStream_t stream = static_cast<cudaStream_t>(cudaStream);

    size_t vertexBufferSize = vertexCount * 3 * sizeof(float);
    size_t normalBufferSize = vertexCount * 3 * sizeof(float);
    size_t indexBufferSize = vertexCount * sizeof(unsigned int);

    std::cout << "[MeshManager] Updating mesh '" << meshId << "' from GPU: "
              << vertexCount << " vertices ("
              << (vertexBufferSize / 1024.0 / 1024.0) << " MB)" << std::endl;

    glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo_vertices);
    glBufferData(GL_ARRAY_BUFFER, vertexBufferSize, nullptr, GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo_normals);
    glBufferData(GL_ARRAY_BUFFER, normalBufferSize, nullptr, GL_DYNAMIC_DRAW);

    cudaGraphicsResource_t vbo_resource, nbo_resource;
    cudaError_t err;

    err = cudaGraphicsGLRegisterBuffer(
        &vbo_resource,
        mesh.vbo_vertices,
        cudaGraphicsRegisterFlagsWriteDiscard);
    if (err != cudaSuccess) {
        std::cerr << "[MeshManager] Failed to register VBO: "
                  << cudaGetErrorString(err) << std::endl;
        return nullptr;
    }

    err = cudaGraphicsGLRegisterBuffer(
        &nbo_resource,
        mesh.vbo_normals,
        cudaGraphicsRegisterFlagsWriteDiscard);
    if (err != cudaSuccess) {
        std::cerr << "[MeshManager] Failed to register NBO: "
                  << cudaGetErrorString(err) << std::endl;
        cudaGraphicsUnregisterResource(vbo_resource);
        return nullptr;
    }

    err = cudaGraphicsMapResources(1, &vbo_resource, stream);
    if (err != cudaSuccess) {
        std::cerr << "[MeshManager] Failed to map VBO: "
                  << cudaGetErrorString(err) << std::endl;
        cudaGraphicsUnregisterResource(vbo_resource);
        cudaGraphicsUnregisterResource(nbo_resource);
        return nullptr;
    }

    err = cudaGraphicsMapResources(1, &nbo_resource, stream);
    if (err != cudaSuccess) {
        std::cerr << "[MeshManager] Failed to map NBO: "
                  << cudaGetErrorString(err) << std::endl;
        cudaGraphicsUnmapResources(1, &vbo_resource, stream);
        cudaGraphicsUnregisterResource(vbo_resource);
        cudaGraphicsUnregisterResource(nbo_resource);
        return nullptr;
    }

    void* d_vbo_ptr = nullptr;
    void* d_nbo_ptr = nullptr;
    size_t vbo_size = 0;
    size_t nbo_size = 0;

    cudaGraphicsResourceGetMappedPointer(&d_vbo_ptr, &vbo_size, vbo_resource);
    cudaGraphicsResourceGetMappedPointer(&d_nbo_ptr, &nbo_size, nbo_resource);

    err = cudaMemcpyAsync(
        d_vbo_ptr,
        d_vertices,
        vertexBufferSize,
        cudaMemcpyDeviceToDevice,
        stream);
    if (err != cudaSuccess) {
        std::cerr << "[MeshManager] Failed to copy vertices D2D: "
                  << cudaGetErrorString(err) << std::endl;
    }

    err = cudaMemcpyAsync(
        d_nbo_ptr,
        d_normals,
        normalBufferSize,
        cudaMemcpyDeviceToDevice,
        stream);
    if (err != cudaSuccess) {
        std::cerr << "[MeshManager] Failed to copy normals D2D: "
                  << cudaGetErrorString(err) << std::endl;
    }

    cudaGraphicsUnmapResources(1, &vbo_resource, stream);
    cudaGraphicsUnmapResources(1, &nbo_resource, stream);
    cudaGraphicsUnregisterResource(vbo_resource);
    cudaGraphicsUnregisterResource(nbo_resource);

    std::vector<unsigned int> indices(vertexCount);
    for (size_t i = 0; i < vertexCount; ++i) {
        indices[i] = static_cast<unsigned int>(i);
    }

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.ebo);
    glBufferData(
        GL_ELEMENT_ARRAY_BUFFER,
        indexBufferSize,
        indices.data(),
        GL_STATIC_DRAW);

    glBindVertexArray(mesh.vao);

    glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo_vertices);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo_normals);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.ebo);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    mesh.index_count = static_cast<GLsizei>(vertexCount);

    std::cout << "[MeshManager] ✅ D2D mesh update complete: "
              << (vertexCount / 3) << " triangles" << std::endl;

    return &mesh;
}

} // namespace Urbaxio::Engine


