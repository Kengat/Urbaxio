#pragma once

#include <string>
#include <glm/glm.hpp>
#include <cstdint>

namespace Urbaxio::Engine {
	// Represents the visual properties of a surface.
	struct Material {
		std::string name;
		glm::vec3 diffuseColor = glm::vec3(0.8f);
		// Path to the diffuse texture file.
		std::string diffuseTexturePath;
		// OpenGL texture ID. 0 means no texture is loaded to the GPU.
		uint32_t diffuseTextureID = 0;
		// Future properties can be added here:
		// float roughness = 0.5f;
		// float metalness = 0.0f;
		// uint32_t normalTextureID = 0;
	};
} // namespace Urbaxio::Engine


