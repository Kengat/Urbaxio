// shell/include/file_io.h
#pragma once

#include <string>
#include <vector>
#include <atomic>
#include <cad_kernel/MeshBuffers.h>
#include <engine/MeshGroup.h>
#include <engine/Material.h>

// Forward declare scene
namespace Urbaxio::Engine { class Scene; }

namespace Urbaxio::FileIO {

struct LoadedObjectData {
    std::string name;
    CadKernel::MeshBuffers mesh;
    std::vector<Engine::MeshGroup> meshGroups;
};

struct LoadedSceneData {
    std::vector<Engine::Material> materials;
    std::vector<LoadedObjectData> objects;
};

bool ExportSceneToObj(const std::string& filepath, const Engine::Scene& scene);

LoadedSceneData LoadObjToIntermediate(const std::string& filepath, float scale, std::atomic<float>& progress);

void ApplyLoadedDataToScene(const LoadedSceneData& data, Engine::Scene& scene);

CadKernel::MeshBuffers LoadMeshFromObj(const std::string& filepath);

}

