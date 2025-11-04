#include "engine/MaterialManager.h"
#include <glm/glm.hpp>
#include <iostream>

namespace Urbaxio::Engine {

	MaterialManager::MaterialManager() {
		// Add a default material upon creation.
		ClearMaterials();
	}

	void MaterialManager::AddMaterial(const Material& material) {
		materials_[material.name] = material;
	}

	Material* MaterialManager::GetMaterial(const std::string& name) {
		auto it = materials_.find(name);
		if (it != materials_.end()) {
			return &it->second;
		}
		// --- MODIFIED: Add a warning and fallback ---
		if (name != "Default") {
			std::cerr << "MaterialManager Warning: Material '" << name << "' not found. Falling back to 'Default'." << std::endl;
		}
		// Fallback to default material if the requested one is not found
		auto default_it = materials_.find("Default");
		if (default_it != materials_.end()) {
			return &default_it->second;
		}
		return nullptr; // Should not happen if ClearMaterials was called
	}

	const Material* MaterialManager::GetMaterial(const std::string& name) const {
		auto it = materials_.find(name);
		if (it != materials_.end()) {
			return &it->second;
		}
		if (name != "Default") {
			std::cerr << "MaterialManager Warning: Material '" << name << "' not found. Falling back to 'Default'." << std::endl;
		}
		auto default_it = materials_.find("Default");
		if (default_it != materials_.end()) {
			return &default_it->second;
		}
		return nullptr;
	}

	std::map<std::string, Material>& MaterialManager::GetAllMaterials() {
		return materials_;
	}

	const std::map<std::string, Material>& MaterialManager::GetAllMaterials() const {
		return materials_;
	}

	void MaterialManager::ClearMaterials() {
		materials_.clear();
		// Ensure a default material always exists.
		AddMaterial({ "Default", glm::vec3(0.8f, 0.8f, 0.8f), "", 0 });
	}

} // namespace Urbaxio::Engine


