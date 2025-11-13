#include "engine/MeshManager.h"
#include <iostream>
#include <glm/glm.hpp>

namespace Urbaxio::Engine {

MeshManager::~MeshManager() {
    Clear();
}

void MeshManager::freeGpuResources(GpuMesh& mesh) {
    if (mesh.vao != 0) { glDeleteVertexArrays(1, &mesh.vao); }
    if (mesh.vbo_vertices != 0) { glDeleteBuffers(1, &mesh.vbo_vertices); }
    if (mesh.vbo_normals != 0) { glDeleteBuffers(1, &mesh.vbo_normals); }
    if (mesh.vbo_uvs != 0) { glDeleteBuffers(1, &mesh.vbo_uvs); }
    if (mesh.ebo != 0) { glDeleteBuffers(1, &mesh.ebo); }
    if (mesh.vbo_instances != 0) { glDeleteBuffers(1, &mesh.vbo_instances); }
    mesh = {}; // Reset to default values
}

GpuMesh* MeshManager::AddMesh(const std::string& meshId, const CadKernel::MeshBuffers& meshData) {
    if (meshData.isEmpty() || meshData.normals.empty()) {
        std::cerr << "MeshManager Error: Cannot add mesh '" << meshId << "' with empty or invalid data." << std::endl;
        return nullptr;
    }
    
    // If a mesh with this ID already exists, clean up its old GPU resources first.
    auto it = meshes_.find(meshId);
    if (it != meshes_.end()) {
        freeGpuResources(it->second);
    }

    GpuMesh newGpuMesh;
    
    glGenVertexArrays(1, &newGpuMesh.vao);
    glBindVertexArray(newGpuMesh.vao);

    glGenBuffers(1, &newGpuMesh.vbo_vertices);
    glBindBuffer(GL_ARRAY_BUFFER, newGpuMesh.vbo_vertices);
    glBufferData(GL_ARRAY_BUFFER, meshData.vertices.size() * sizeof(float), meshData.vertices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glGenBuffers(1, &newGpuMesh.vbo_normals);
    glBindBuffer(GL_ARRAY_BUFFER, newGpuMesh.vbo_normals);
    glBufferData(GL_ARRAY_BUFFER, meshData.normals.size() * sizeof(float), meshData.normals.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);

    if (!meshData.uvs.empty()) {
        glGenBuffers(1, &newGpuMesh.vbo_uvs);
        glBindBuffer(GL_ARRAY_BUFFER, newGpuMesh.vbo_uvs);
        glBufferData(GL_ARRAY_BUFFER, meshData.uvs.size() * sizeof(float), meshData.uvs.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(2);
    }
    
    glGenBuffers(1, &newGpuMesh.ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, newGpuMesh.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, meshData.indices.size() * sizeof(unsigned int), meshData.indices.data(), GL_STATIC_DRAW);
    newGpuMesh.index_count = static_cast<GLsizei>(meshData.indices.size());
    
    // The instance VBO is not created here; it's managed by the renderer
    // because the number of instances can change each frame.
    // However, we can pre-generate it here for cleanliness.
    glGenBuffers(1, &newGpuMesh.vbo_instances);
    glBindBuffer(GL_ARRAY_BUFFER, newGpuMesh.vbo_instances);
    // Setup vertex attributes for the 4x4 instance model matrix
    // A mat4 is four vec4s.
    for (int i = 0; i < 4; ++i) {
        GLuint attribLocation = 3 + i; // Start attributes at location 3
        glEnableVertexAttribArray(attribLocation);
        glVertexAttribPointer(attribLocation, 4, GL_FLOAT, GL_FALSE, sizeof(glm::mat4), (void*)(sizeof(glm::vec4) * i));
        glVertexAttribDivisor(attribLocation, 1); // This is the key to instancing
    }
    
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    meshes_[meshId] = newGpuMesh;
    std::cout << "MeshManager: Added mesh '" << meshId << "' to GPU." << std::endl;
    return &meshes_[meshId];
}

GpuMesh* MeshManager::GetMesh(const std::string& meshId) {
    auto it = meshes_.find(meshId);
    return (it != meshes_.end()) ? &it->second : nullptr;
}

const GpuMesh* MeshManager::GetMesh(const std::string& meshId) const {
    auto it = meshes_.find(meshId);
    return (it != meshes_.end()) ? &it->second : nullptr;
}

void MeshManager::DeleteMesh(const std::string& meshId) {
    auto it = meshes_.find(meshId);
    if (it != meshes_.end()) {
        freeGpuResources(it->second);
        meshes_.erase(it);
    }
}

void MeshManager::Clear() {
    for (auto& pair : meshes_) {
        freeGpuResources(pair.second);
    }
    meshes_.clear();
}

} // namespace Urbaxio::Engine


