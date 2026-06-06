#include "engine/commands/PushPullCommand.h"
#include "engine/scene.h"
#include "engine/scene_object.h"
#include "engine/geometry/BRepGeometry.h"
#include "engine/geometry/MeshGeometry.h"
#include <cad_kernel/cad_kernel.h>
#include <cad_kernel/MeshBuffers.h>

#include <TopoDS_Shape.hxx>
#include <BinTools.hxx>
#include <sstream>
#include <iostream>

namespace Urbaxio::Engine {

PushPullCommand::PushPullCommand(
    Scene* scene, 
    const std::vector<glm::vec3>& faceVertices,
    const glm::vec3& faceNormal,
    float distance,
    bool disableMerge)
    : scene_(scene), 
      faceVertices_(faceVertices),
      faceNormal_(faceNormal),
      distance_(distance),
      disableMerge_(disableMerge)
{
}

const char* PushPullCommand::GetName() const {
    return "Push/Pull";
}

void PushPullCommand::Execute() {
    if (isExecuted_) { // This is a REDO operation
        if (isMeshMode_) {
            if (!meshAfter_.empty() && targetObjectId_ != 0) {
                RestoreMeshGeometry(targetObjectId_, meshAfter_);
            }
        } else {
            if (!shapeAfter_.empty() && targetObjectId_ != 0) {
                RestoreObjectShape(targetObjectId_, shapeAfter_);
            }
        }
        return;
    }

    // This is the FIRST execution of the command
    // Find target object - check both BRep and Mesh objects
    SceneObject* object = nullptr;

    // First try FindObjectByFace (works for BRep)
    object = scene_->FindObjectByFace(faceVertices_);

    // If not found, search mesh objects by vertex positions
    if (!object) {
        for (auto* obj : scene_->get_all_objects()) {
            if (!obj || !obj->hasGeometry()) continue;
            auto* meshGeom = dynamic_cast<MeshGeometry*>(obj->getGeometry());
            if (!meshGeom) continue;

            const auto& mesh = meshGeom->getMesh();
            size_t numVertices = mesh.vertices.size() / 3;
            int matchCount = 0;

            for (const auto& faceVert : faceVertices_) {
                for (size_t i = 0; i < numVertices; ++i) {
                    glm::vec3 meshVert(mesh.vertices[i * 3],
                                       mesh.vertices[i * 3 + 1],
                                       mesh.vertices[i * 3 + 2]);
                    if (glm::distance(faceVert, meshVert) < 1e-4f) {
                        matchCount++;
                        break;
                    }
                }
            }

            // If most face vertices are found in this mesh, it's the target
            if (matchCount >= static_cast<int>(faceVertices_.size()) * 0.8) {
                object = obj;
                break;
            }
        }
    }

    if (!object) {
        std::cerr << "PushPullCommand Error: Could not find an object containing the specified face." << std::endl;
        return;
    }

    targetObjectId_ = object->get_id();

    // Check if this is a MeshGeometry object
    auto* meshGeom = dynamic_cast<MeshGeometry*>(object->getGeometry());
    if (meshGeom) {
        isMeshMode_ = true;
        meshBefore_ = meshGeom->serialize();
        if (meshBefore_.empty()) {
            std::cerr << "PushPullCommand Error: Failed to serialize mesh 'before' state." << std::endl;
            return;
        }

        bool success = scene_->ExtrudeFace(targetObjectId_, faceVertices_, faceNormal_, distance_, disableMerge_);

        if (success) {
            auto* updatedGeom = dynamic_cast<MeshGeometry*>(object->getGeometry());
            if (updatedGeom) {
                meshAfter_ = updatedGeom->serialize();
            }
        } else {
            std::cerr << "PushPullCommand: ExtrudeFace (mesh mode) failed. Restoring." << std::endl;
            RestoreMeshGeometry(targetObjectId_, meshBefore_);
        }

        isExecuted_ = true;
        return;
    }

    // BRep mode (original logic)
    auto* brepGeom = dynamic_cast<BRepGeometry*>(object->getGeometry());
    if (!brepGeom || !brepGeom->getShape()) {
        std::cerr << "PushPullCommand Error: Target object is not a valid B-Rep or Mesh." << std::endl;
        return;
    }

    isMeshMode_ = false;
    shapeBefore_ = SerializeShape(*brepGeom->getShape());
    if (shapeBefore_.empty()) {
        std::cerr << "PushPullCommand Error: Failed to serialize 'before' state." << std::endl;
        return;
    }

    bool success = scene_->ExtrudeFace(targetObjectId_, faceVertices_, faceNormal_, distance_, disableMerge_);

    if (success) {
        auto* updatedGeom = dynamic_cast<BRepGeometry*>(object->getGeometry());
        if (updatedGeom && updatedGeom->getShape()) {
            shapeAfter_ = SerializeShape(*updatedGeom->getShape());
        }
    } else {
        std::cerr << "PushPullCommand: ExtrudeFace operation failed. Aborting." << std::endl;
        RestoreObjectShape(targetObjectId_, shapeBefore_);
    }

    isExecuted_ = true;
}

void PushPullCommand::Undo() {
    if (targetObjectId_ == 0) {
        std::cerr << "PushPullCommand Error: Cannot undo, no target object." << std::endl;
        return;
    }

    if (isMeshMode_) {
        if (meshBefore_.empty()) {
            std::cerr << "PushPullCommand Error: Cannot undo, mesh 'before' state was not captured." << std::endl;
            return;
        }
        RestoreMeshGeometry(targetObjectId_, meshBefore_);
    } else {
        if (shapeBefore_.empty()) {
            std::cerr << "PushPullCommand Error: Cannot undo, 'before' state was not properly captured." << std::endl;
            return;
        }
        RestoreObjectShape(targetObjectId_, shapeBefore_);
    }
}

void PushPullCommand::RestoreObjectShape(uint64_t objectId, const std::vector<char>& shapeData) {
    SceneObject* object = scene_->get_object_by_id(objectId);
    if (!object) {
        std::cerr << "PushPullCommand Warning: Could not find object " << objectId << " to restore shape." << std::endl;
        return;
    }
    
    TopoDS_Shape restoredShape;
    std::stringstream ss(std::string(shapeData.begin(), shapeData.end()));
    try {
        BinTools::Read(restoredShape, ss);
    } catch (...) {
        std::cerr << "PushPullCommand Error: Failed to deserialize shape data." << std::endl;
        return;
    }
    
    if (restoredShape.IsNull()) {
        std::cerr << "PushPullCommand Error: Deserialized shape is null." << std::endl;
        return;
    }

    // NEW: Update via the new geometry system
    auto geometry = std::make_unique<BRepGeometry>(CadKernel::OCCT_ShapeUniquePtr(new TopoDS_Shape(restoredShape)));
    object->setGeometry(std::move(geometry));
    scene_->UpdateObjectBoundary(object);

    // Mark the scene dirty to force the renderer to re-compile the static batch
    scene_->MarkStaticGeometryDirty();
}

std::vector<char> PushPullCommand::SerializeShape(const TopoDS_Shape& shape) {
    std::stringstream ss;
    try {
        BinTools::Write(shape, ss);
        std::string const& s = ss.str();
        return std::vector<char>(s.begin(), s.end());
    } catch (...) {
        std::cerr << "PushPullCommand Error: Exception during shape serialization." << std::endl;
        return {};
    }
}

void PushPullCommand::RestoreMeshGeometry(uint64_t objectId, const std::vector<char>& meshData) {
    SceneObject* object = scene_->get_object_by_id(objectId);
    if (!object) {
        std::cerr << "PushPullCommand Warning: Could not find object " << objectId << " to restore mesh." << std::endl;
        return;
    }

    auto restoredGeom = MeshGeometry::deserialize(meshData);
    if (!restoredGeom) {
        std::cerr << "PushPullCommand Error: Failed to deserialize mesh data." << std::endl;
        return;
    }

    object->setGeometry(std::move(restoredGeom));
    scene_->UpdateObjectBoundary(object);
    scene_->MarkStaticGeometryDirty();
}

} // namespace Urbaxio::Engine 