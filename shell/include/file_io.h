// shell/include/file_io.h
#pragma once

#include <string>

// Forward declare scene and mesh buffers
namespace Urbaxio::Engine { class Scene; }
namespace Urbaxio::CadKernel { struct MeshBuffers; }

namespace Urbaxio::FileIO {

bool ExportSceneToObj(const std::string& filepath, const Engine::Scene& scene);
bool ImportObjToScene(const std::string& filepath, Engine::Scene& scene, float scale);

// --- NEW: Low-level loader for internal resources ---
CadKernel::MeshBuffers LoadMeshFromObj(const std::string& filepath);

}

