#pragma once

#include <cad_kernel/MeshBuffers.h>

#include <functional>
#include <string_view>

namespace Urbaxio::Engine {

	inline size_t GenerateMeshHash(const CadKernel::MeshBuffers& meshData) {
		std::hash<std::string_view> hasher;
		size_t h1 = hasher(std::string_view(reinterpret_cast<const char*>(meshData.vertices.data()), meshData.vertices.size() * sizeof(float)));
		size_t h2 = hasher(std::string_view(reinterpret_cast<const char*>(meshData.normals.data()), meshData.normals.size() * sizeof(float)));
		size_t h3 = hasher(std::string_view(reinterpret_cast<const char*>(meshData.indices.data()), meshData.indices.size() * sizeof(unsigned int)));
		return h1 ^ (h2 << 1) ^ (h3 << 2);
	}

} // namespace Urbaxio::Engine


