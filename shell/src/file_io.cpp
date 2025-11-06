// shell/src/file_io.cpp
#include "../include/file_io.h"
#include <engine/scene.h>
#include <engine/scene_object.h>
#include <engine/geometry/MeshGeometry.h>
#include <engine/geometry/BRepGeometry.h>
#include <engine/MaterialManager.h>
#include <engine/MeshGroup.h>
#include <cad_kernel/MeshBuffers.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <filesystem>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"
#include <string>
#include <algorithm>

namespace {
    // Custom comparator for tinyobj::index_t to use it in a map
    struct IndexComparator {
        bool operator()(const tinyobj::index_t& a, const tinyobj::index_t& b) const {
            if (a.vertex_index < b.vertex_index) return true;
            if (a.vertex_index > b.vertex_index) return false;
            if (a.normal_index < b.normal_index) return true;
            if (a.normal_index > b.normal_index) return false;
            if (a.texcoord_index < b.texcoord_index) return true;
            if (a.texcoord_index > b.texcoord_index) return false;
            return false;
        }
    };
    // --- NEW: String trimming helper function ---
    std::string trim(const std::string& str) {
        const char* whitespace = " \t\n\r\f\v";
        size_t first = str.find_first_not_of(whitespace);
        if (std::string::npos == first) {
            return str;
        }
        size_t last = str.find_last_not_of(whitespace);
        return str.substr(first, (last - first + 1));
    }
}

namespace Urbaxio::FileIO {

bool ExportSceneToObj(const std::string& filepath, const Engine::Scene& scene) {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "FileIO Error: Could not open " << filepath << " for writing." << std::endl;
        return false;
    }

    file << "# Urbaxio OBJ Export\n";

    // --- NEW: Transformation from our Z-Up to standard Y-Up for export ---
    const float exportScale = 1.0f; // Changed to 1.0, scale should be handled by user
    glm::mat4 z_up_to_y_up = glm::rotate(glm::mat4(1.0f), glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    glm::mat3 normal_transform = glm::mat3(z_up_to_y_up);

    size_t vertex_offset = 1; // OBJ indices are 1-based
    size_t normal_offset = 1;
    size_t uv_offset = 1;

    for (const auto* obj : scene.get_all_objects()) {
        // We only export objects that are marked as exportable and have a mesh
        if (!obj || !obj->isExportable() || !obj->hasMesh()) {
            continue;
        }

        const auto& mesh = obj->getMeshBuffers(); // Use the caching getter
        if (mesh.isEmpty()) continue;
        file << "o " << obj->get_name() << "_" << obj->get_id() << "\n";

        // Write vertices
        for (size_t i = 0; i < mesh.vertices.size(); i += 3) {
            glm::vec3 pos(mesh.vertices[i], mesh.vertices[i+1], mesh.vertices[i+2]);
            pos *= exportScale; // Apply scale first
            glm::vec3 transformed_pos = z_up_to_y_up * glm::vec4(pos, 1.0);
            file << "v " << transformed_pos.x << " " << transformed_pos.y << " " << transformed_pos.z << "\n";
        }

        // Write normals
        for (size_t i = 0; i < mesh.normals.size(); i += 3) {
            glm::vec3 norm(mesh.normals[i], mesh.normals[i+1], mesh.normals[i+2]);
            // Normals are only rotated, not scaled
            glm::vec3 transformed_norm = normal_transform * norm;
            file << "vn " << transformed_norm.x << " " << transformed_norm.y << " " << transformed_norm.z << "\n";
        }

        // Write UVs if they exist
        if (!mesh.uvs.empty()) {
            for (size_t i = 0; i < mesh.uvs.size(); i += 2) {
                file << "vt " << mesh.uvs[i] << " " << mesh.uvs[i+1] << "\n";
            }
        }

        for (const auto& group : obj->meshGroups) {
            file << "usemtl " << group.materialName << "\n";
            for (size_t i = group.startIndex; i < group.startIndex + group.indexCount; i += 3) {
                file << "f";
                for (int j = 0; j < 3; ++j) {
                    unsigned int v_idx = mesh.indices[i+j] + vertex_offset;
                    unsigned int n_idx = mesh.indices[i+j] + normal_offset;
                    file << " " << v_idx;
                    if (!mesh.uvs.empty()) {
                        unsigned int uv_idx = mesh.indices[i+j] + uv_offset;
                        file << "/" << uv_idx;
                    }
                    file << "/" << n_idx;
                }
                file << "\n";
            }
        }

        vertex_offset += mesh.vertices.size() / 3;
        normal_offset += mesh.normals.size() / 3;
        if (!mesh.uvs.empty()) {
            uv_offset += mesh.uvs.size() / 2;
        }
    }

    std::cout << "FileIO: Successfully exported scene to " << filepath << std::endl;
    return true;
}

LoadedSceneData LoadObjToIntermediate(const std::string& filepath, float scale, std::atomic<float>& progress) {
    LoadedSceneData loadedData;
    progress = 0.0f;

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;

    std::filesystem::path path(filepath);
    std::string mtl_base_dir = path.parent_path().string();

    if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, filepath.c_str(), mtl_base_dir.c_str(), true)) {
        std::cerr << "FileIO Error: Failed to load OBJ file: " << warn << err << std::endl;
        progress = 1.0f;
        return loadedData;
    }

    if (!warn.empty()) {
        std::cout << "FileIO Warning: " << warn << std::endl;
    }

    progress = 0.1f;

    for (const auto& mat : materials) {
        Engine::Material newMat;
        newMat.name = trim(mat.name);
        newMat.diffuseColor = { mat.diffuse[0], mat.diffuse[1], mat.diffuse[2] };
        if (!mat.diffuse_texname.empty()) {
            std::filesystem::path texture_path = std::filesystem::path(mtl_base_dir) / trim(mat.diffuse_texname);
            newMat.diffuseTexturePath = std::filesystem::absolute(texture_path).string();
        }
        loadedData.materials.push_back(newMat);
    }

    progress = 0.2f;

    glm::mat4 y_up_to_z_up = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    glm::mat3 normal_transform = glm::mat3(y_up_to_z_up);

    float shape_progress_step = shapes.empty() ? 0.8f : (0.8f / shapes.size());
    for (size_t s = 0; s < shapes.size(); ++s) {
        const auto& shape = shapes[s];

        LoadedObjectData newObjectData;
        CadKernel::MeshBuffers combinedMesh;
        std::vector<Engine::MeshGroup> meshGroups;
        std::map<tinyobj::index_t, uint32_t, IndexComparator> unique_vertices;
        std::map<int, std::vector<tinyobj::index_t>> facesByMaterial;
        for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); ++f) {
            int material_id = shape.mesh.material_ids[f];
            for (int v = 0; v < 3; ++v) {
                facesByMaterial[material_id].push_back(shape.mesh.indices[f * 3 + v]);
            }
        }

        for(const auto& [material_id, indices] : facesByMaterial) {
            Engine::MeshGroup group;
            group.startIndex = combinedMesh.indices.size();

            if (material_id >= 0 && material_id < materials.size()) {
                group.materialName = trim(materials[material_id].name);
            } else {
                group.materialName = "Default";
            }

            for (const auto& index : indices) {
                auto it = unique_vertices.find(index);
                if (it != unique_vertices.end()) {
                    combinedMesh.indices.push_back(it->second);
                } else {
                    uint32_t new_idx = static_cast<uint32_t>(combinedMesh.vertices.size() / 3);

                    glm::vec3 pos( attrib.vertices[3 * index.vertex_index + 0], attrib.vertices[3 * index.vertex_index + 1], attrib.vertices[3 * index.vertex_index + 2] );
                    glm::vec3 transformed_pos = y_up_to_z_up * glm::vec4(pos, 1.0f);
                    transformed_pos *= scale;
                    combinedMesh.vertices.push_back(transformed_pos.x); combinedMesh.vertices.push_back(transformed_pos.y); combinedMesh.vertices.push_back(transformed_pos.z);

                    if (index.normal_index >= 0) {
                        glm::vec3 norm( attrib.normals[3 * index.normal_index + 0], attrib.normals[3 * index.normal_index + 1], attrib.normals[3 * index.normal_index + 2] );
                        glm::vec3 transformed_norm = normal_transform * norm;
                        combinedMesh.normals.push_back(transformed_norm.x); combinedMesh.normals.push_back(transformed_norm.y); combinedMesh.normals.push_back(transformed_norm.z);
                    } else { combinedMesh.normals.insert(combinedMesh.normals.end(), {0,0,1}); }

                    if (index.texcoord_index >= 0) {
                        combinedMesh.uvs.push_back(attrib.texcoords[2 * index.texcoord_index + 0]); combinedMesh.uvs.push_back(attrib.texcoords[2 * index.texcoord_index + 1]);
                    } else { combinedMesh.uvs.insert(combinedMesh.uvs.end(), {0,0}); }

                    combinedMesh.indices.push_back(new_idx);
                    unique_vertices[index] = new_idx;
                }
            }
            group.indexCount = combinedMesh.indices.size() - group.startIndex;
            if (group.indexCount > 0) {
                meshGroups.push_back(group);
            }
        }

        newObjectData.name = shape.name;
        if (newObjectData.name.empty()) {
            newObjectData.name = path.stem().string();
        }

        newObjectData.mesh = combinedMesh;
        newObjectData.meshGroups = meshGroups;
        loadedData.objects.push_back(std::move(newObjectData));

        progress = 0.2f + (float)(s + 1) * shape_progress_step;
    }

    std::cout << "FileIO: Successfully parsed " << shapes.size() << " shape(s) from " << filepath << " in worker thread." << std::endl;
    progress = 1.0f;
    return loadedData;
}

void ApplyLoadedDataToScene(const LoadedSceneData& data, Engine::Scene& scene) {
    for (const auto& mat : data.materials) {
        scene.getMaterialManager()->AddMaterial(mat);
    }

    for (const auto& objectData : data.objects) {
        Engine::SceneObject* new_obj = scene.create_object(objectData.name);
        if (new_obj) {
            // Step 1: Set the geometry. This will call invalidateMeshCache() internally,
            // which now correctly clears the meshGroups vector.
            auto mesh_geom = std::make_unique<Engine::MeshGeometry>(objectData.mesh);
            new_obj->setGeometry(std::move(mesh_geom));
            
            // Step 2: NOW, after the geometry is set and the old groups are cleared,
            // we assign the correct mesh groups that we loaded from the OBJ file.
            // The next call to getMeshBuffers() will use these correct groups.
            new_obj->meshGroups = objectData.meshGroups;
        }
    }

    std::cout << "FileIO: Applied " << data.objects.size() << " object(s) to the scene." << std::endl;
}

CadKernel::MeshBuffers LoadMeshFromObj(const std::string& filepath) {
    Urbaxio::CadKernel::MeshBuffers resultMesh;

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;

    if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, filepath.c_str())) {
        std::cerr << "FileIO Error (LoadMeshFromObj): Failed to load OBJ file: " << filepath << " - " << warn << err << std::endl;
        return resultMesh; // Return empty mesh on failure
    }

    // Transformation from standard Y-Up to our Z-Up
    glm::mat4 y_up_to_z_up = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    glm::mat3 normal_transform = glm::mat3(y_up_to_z_up);

    std::map<tinyobj::index_t, uint32_t, IndexComparator> unique_vertices;
    // Combine all shapes from the OBJ file into a single mesh
    for (const auto& shape : shapes) {
        for (const auto& index : shape.mesh.indices) {
            auto it = unique_vertices.find(index);
            if (it != unique_vertices.end()) {
                resultMesh.indices.push_back(it->second);
            } else {
                uint32_t new_idx = static_cast<uint32_t>(resultMesh.vertices.size() / 3);
                
                // Vertex data
                glm::vec3 pos(
                    attrib.vertices[3 * index.vertex_index + 0],
                    attrib.vertices[3 * index.vertex_index + 1],
                    attrib.vertices[3 * index.vertex_index + 2]
                );
                glm::vec3 transformed_pos = y_up_to_z_up * glm::vec4(pos, 1.0f);
                resultMesh.vertices.push_back(transformed_pos.x);
                resultMesh.vertices.push_back(transformed_pos.y);
                resultMesh.vertices.push_back(transformed_pos.z);
                // Normal data
                if (index.normal_index >= 0) {
                    glm::vec3 norm(
                        attrib.normals[3 * index.normal_index + 0],
                        attrib.normals[3 * index.normal_index + 1],
                        attrib.normals[3 * index.normal_index + 2]
                    );
                    glm::vec3 transformed_norm = normal_transform * norm;
                    resultMesh.normals.push_back(transformed_norm.x);
                    resultMesh.normals.push_back(transformed_norm.y);
                    resultMesh.normals.push_back(transformed_norm.z);
                } else { resultMesh.normals.insert(resultMesh.normals.end(), {0,0,1}); }
                
                // UV data
                if (index.texcoord_index >= 0) {
                    resultMesh.uvs.push_back(attrib.texcoords[2 * index.texcoord_index + 0]);
                    resultMesh.uvs.push_back(attrib.texcoords[2 * index.texcoord_index + 1]);
                } else { resultMesh.uvs.insert(resultMesh.uvs.end(), {0,0}); }
                resultMesh.indices.push_back(new_idx);
                unique_vertices[index] = new_idx;
            }
        }
    }

    return resultMesh;
}

} // namespace Urbaxio::FileIO

