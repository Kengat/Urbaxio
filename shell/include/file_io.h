// shell/include/file_io.h
#pragma once

#include <string>

// Forward declare scene
namespace Urbaxio::Engine { class Scene; }

namespace Urbaxio::FileIO {

bool ExportSceneToObj(const std::string& filepath, const Engine::Scene& scene);
bool ImportObjToScene(const std::string& filepath, Engine::Scene& scene, float scale);

}

