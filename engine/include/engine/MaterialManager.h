#pragma once

#include "engine/Material.h"
#include <string>
#include <map>
#include <memory>

namespace Urbaxio::Engine {
	class MaterialManager {
	public:
		MaterialManager();
		// Adds a new material or overwrites an existing one with the same name.
		void AddMaterial(const Material& material);
		// Retrieves a pointer to a material by its name. Returns nullptr if not found.
		Material* GetMaterial(const std::string& name);
		// Retrieves a const pointer to a material by its name.
		const Material* GetMaterial(const std::string& name) const;
        // Returns a reference to the entire map of materials.
        std::map<std::string, Material>& GetAllMaterials();
        // Returns a const reference to the entire map of materials.
        const std::map<std::string, Material>& GetAllMaterials() const;
		// Clears all materials and resets to the default state.
		void ClearMaterials();
	private:
		std::map<std::string, Material> materials_;
	};
} // namespace Urbaxio::Engine


