// shell/src/file_io.cpp
#include "../include/file_io.h"
#include <engine/scene.h>
#include <engine/scene_object.h>
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

namespace Urbaxio::FileIO {

bool ExportSceneToObj(const std::string& filepath, const Engine::Scene& scene) {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "FileIO Error: Could not open " << filepath << " for writing." << std::endl;
        return false;
    }

    file << "# Urbaxio OBJ Export\n";

    // --- NEW: Transformation from our Z-Up to standard Y-Up for export ---
    const float exportScale = 100.0f; // Scale up (e.g., from meters to cm)
    glm::mat4 z_up_to_y_up = glm::rotate(glm::mat4(1.0f), glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    glm::mat3 normal_transform = glm::mat3(z_up_to_y_up);

    size_t vertex_offset = 1; // OBJ indices are 1-based
    size_t normal_offset = 1;

    for (const auto* obj : scene.get_all_objects()) {
        // We only export objects that are marked as exportable and have a mesh
        if (!obj || !obj->isExportable() || !obj->has_mesh() || obj->get_mesh_buffers().isEmpty()) {
            continue;
        }

        const auto& mesh = obj->get_mesh_buffers();
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

        // Write faces (format: f v1//vn1 v2//vn2 v3//vn3)
        for (size_t i = 0; i < mesh.indices.size(); i += 3) {
            unsigned int i0 = mesh.indices[i] + vertex_offset;
            unsigned int i1 = mesh.indices[i+1] + vertex_offset;
            unsigned int i2 = mesh.indices[i+2] + vertex_offset;
            // In our case, vertex and normal indices are the same
            unsigned int n0 = mesh.indices[i] + normal_offset;
            unsigned int n1 = mesh.indices[i+1] + normal_offset;
            unsigned int n2 = mesh.indices[i+2] + normal_offset;
            file << "f " << i0 << "//" << n0 << " " << i1 << "//" << n1 << " " << i2 << "//" << n2 << "\n";
        }

        vertex_offset += mesh.vertices.size() / 3;
        normal_offset += mesh.normals.size() / 3;
    }

    std::cout << "FileIO: Successfully exported scene to " << filepath << std::endl;
    return true;
}

bool ImportObjToScene(const std::string& filepath, Engine::Scene& scene, float scale) {
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;

    if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, filepath.c_str())) {
        std::cerr << "FileIO Error: Failed to load OBJ file: " << warn << err << std::endl;
        return false;
    }

    if (!warn.empty()) {
        std::cout << "FileIO Warning: " << warn << std::endl;
    }

    // --- NEW: Transformation from standard Y-Up to our Z-Up on import ---
    // The scale is now passed as a parameter
    glm::mat4 y_up_to_z_up = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    glm::mat3 normal_transform = glm::mat3(y_up_to_z_up);

    // Import each shape from the OBJ as a separate SceneObject
    for (const auto& shape : shapes) {
        Urbaxio::CadKernel::MeshBuffers mesh;
        const auto& obj_mesh = shape.mesh;

        // The library triangulates faces for us, so we can assume faces are triangles.
        // We can't directly use obj_mesh.indices because vertices/normals might be shared.
        // We build a new, flat list of vertices and normals for our renderer.
        for (const auto& index : obj_mesh.indices) {
            // Vertex data
            glm::vec3 pos(
                attrib.vertices[3 * index.vertex_index + 0],
                attrib.vertices[3 * index.vertex_index + 1],
                attrib.vertices[3 * index.vertex_index + 2]
            );
            glm::vec3 transformed_pos = y_up_to_z_up * glm::vec4(pos, 1.0f);
            transformed_pos *= scale;
            mesh.vertices.push_back(transformed_pos.x);
            mesh.vertices.push_back(transformed_pos.y);
            mesh.vertices.push_back(transformed_pos.z);

            // Normal data (check if available)
            if (index.normal_index >= 0) {
                glm::vec3 norm(
                    attrib.normals[3 * index.normal_index + 0],
                    attrib.normals[3 * index.normal_index + 1],
                    attrib.normals[3 * index.normal_index + 2]
                );
                glm::vec3 transformed_norm = normal_transform * norm;
                mesh.normals.push_back(transformed_norm.x);
                mesh.normals.push_back(transformed_norm.y);
                mesh.normals.push_back(transformed_norm.z);
            } else {
                // If no normals, push a zero vector as a placeholder. A better solution would be to compute them.
                mesh.normals.push_back(0); mesh.normals.push_back(0); mesh.normals.push_back(1);
            }
        }
        
        // tinyobjloader already gives us a flat list of vertices for triangles,
        // so we can just create sequential indices.
        mesh.indices.resize(obj_mesh.indices.size());
        for (size_t i = 0; i < obj_mesh.indices.size(); ++i) {
            mesh.indices[i] = i;
        }

        std::string objectName = shape.name;
        if (objectName.empty()) {
            objectName = "ImportedObject_" + std::filesystem::path(filepath).stem().string();
        }
        
        Engine::SceneObject* new_obj = scene.create_object(objectName);
        if (new_obj) {
            // IMPORTANT: We do NOT set a shape. This is a mesh-only object.
            new_obj->set_mesh_buffers(std::move(mesh));
            new_obj->vao = 0; // Mark for GPU upload
        }
    }

    std::cout << "FileIO: Successfully imported " << shapes.size() << " shape(s) from " << filepath << std::endl;
    return true;
}

} // namespace Urbaxio::FileIO

