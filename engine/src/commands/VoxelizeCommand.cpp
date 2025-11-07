#include "engine/commands/VoxelizeCommand.h"
#include "engine/scene.h"
#include "engine/scene_object.h"
#include "engine/geometry/BRepGeometry.h"
#include "engine/geometry/VolumetricGeometry.h"
#include "engine/geometry/VoxelGrid.h"


#include <iostream>


// OpenCASCADE Includes for Voxelization
#include <TopoDS_Shape.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <gp_Pnt.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Vertex.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <cmath>

// --- NEW: Includes for libigl voxelization ---
#include <cad_kernel/cad_kernel.h>
#include <ShapeFix_Shape.hxx>
#include <igl/signed_distance.h>
#include <Eigen/Core>


namespace { // Anonymous namespace for the helper function


// The core B-Rep to SDF Grid conversion function
std::unique_ptr<Urbaxio::Engine::VoxelGrid> BRepToSDFGrid(const TopoDS_Shape& shape, int resolution) {
    if (shape.IsNull()) {
        return nullptr;
    }

    // --- STEP 1: Heal the B-Rep shape before triangulation ---
    Handle(ShapeFix_Shape) fixer = new ShapeFix_Shape(shape);
    fixer->Perform();
    TopoDS_Shape healedShape = fixer->Shape();

    // 1) Tessellate the B-Rep shape into a triangle mesh
    Urbaxio::CadKernel::MeshBuffers mesh = Urbaxio::CadKernel::TriangulateShape(healedShape);
    if (mesh.isEmpty()) {
        std::cerr << "Voxelize Error: Failed to triangulate B-Rep shape." << std::endl;
        return nullptr;
    }

    // 2) Convert mesh to Eigen matrices
    Eigen::MatrixXd V(mesh.vertices.size() / 3, 3);
    for (size_t i = 0; i < (size_t)V.rows(); ++i) {
        V((Eigen::Index)i, 0) = mesh.vertices[i * 3 + 0];
        V((Eigen::Index)i, 1) = mesh.vertices[i * 3 + 1];
        V((Eigen::Index)i, 2) = mesh.vertices[i * 3 + 2];
    }

    Eigen::MatrixXi F(mesh.indices.size() / 3, 3);
    for (size_t i = 0; i < (size_t)F.rows(); ++i) {
        F((Eigen::Index)i, 0) = (int)mesh.indices[i * 3 + 0];
        F((Eigen::Index)i, 1) = (int)mesh.indices[i * 3 + 1];
        F((Eigen::Index)i, 2) = (int)mesh.indices[i * 3 + 2];
    }

    // 3) Define voxel grid from mesh AABB with padding
    Eigen::Vector3d box_min = V.colwise().minCoeff();
    Eigen::Vector3d box_max = V.colwise().maxCoeff();
    Eigen::Vector3d size = box_max - box_min;
    double padding = size.maxCoeff() * 0.1;
    box_min.array() -= padding;
    box_max.array() += padding;
    size = box_max - box_min;

    double voxelSize = size.maxCoeff() / (double)resolution;
    glm::uvec3 dims((unsigned)std::ceil(size.x() / voxelSize), (unsigned)std::ceil(size.y() / voxelSize), (unsigned)std::ceil(size.z() / voxelSize));
    glm::vec3 origin((float)box_min.x(), (float)box_min.y(), (float)box_min.z());

    // 4) Build query points P
    Eigen::MatrixXd P((Eigen::Index)dims.x * (Eigen::Index)dims.y * (Eigen::Index)dims.z, 3);
    for (unsigned int z = 0; z < dims.z; ++z) {
        for (unsigned int y = 0; y < dims.y; ++y) {
            for (unsigned int x = 0; x < dims.x; ++x) {
                size_t index = (size_t)z * (size_t)dims.x * (size_t)dims.y + (size_t)y * (size_t)dims.x + (size_t)x;
                P((Eigen::Index)index, 0) = origin.x + ((double)x + 0.5) * voxelSize;
                P((Eigen::Index)index, 1) = origin.y + ((double)y + 0.5) * voxelSize;
                P((Eigen::Index)index, 2) = origin.z + ((double)z + 0.5) * voxelSize;
            }
        }
    }

    // 5) Compute signed distances
    Eigen::VectorXd S; Eigen::VectorXi I; Eigen::MatrixXd C; Eigen::MatrixXd N;
    igl::signed_distance(P, V, F, igl::SIGNED_DISTANCE_TYPE_WINDING_NUMBER, S, I, C, N);

    // 6) Fill VoxelGrid
    auto grid = std::make_unique<Urbaxio::Engine::VoxelGrid>(dims, origin, (float)voxelSize);
    const size_t total = (size_t)S.size();
    for (size_t i = 0; i < total; ++i) {
        grid->sdfData[i] = (float)S((Eigen::Index)i);
    }

    std::cout << "Voxelization complete. Grid size: " << dims.x << "x" << dims.y << "x" << dims.z << std::endl;
    return grid;
}


} // end anonymous namespace


namespace Urbaxio::Engine {


VoxelizeCommand::VoxelizeCommand(Scene* scene, uint64_t objectId, int resolution)
    : scene_(scene), objectId_(objectId), resolution_(resolution) {}
VoxelizeCommand::~VoxelizeCommand() = default;


const char* VoxelizeCommand::GetName() const {
    return "Voxelize Object";
}


void VoxelizeCommand::Execute() {
    if (!scene_) return;
    SceneObject* obj = scene_->get_object_by_id(objectId_);
    if (!obj) return;
    
    // If we are re-doing, just swap the stored geometry back in.
    if (voxelizedGeometry_) {
        originalGeometry_ = obj->setGeometry(std::move(voxelizedGeometry_));
        obj->invalidateMeshCache();
        scene_->MarkStaticGeometryDirty();
        return;
    }
    
    auto* brepGeom = dynamic_cast<BRepGeometry*>(obj->getGeometry());
    if (!brepGeom || !brepGeom->getShape()) {
        std::cerr << "VoxelizeCommand Error: Target object is not a B-Rep or has no shape." << std::endl;
        return;
    }
    const TopoDS_Shape* shape = brepGeom->getShape();
    
    std::cout << "Starting voxelization for object " << objectId_ << "..." << std::endl;
    
    auto grid = BRepToSDFGrid(*shape, resolution_);
    
    if (!grid) {
        std::cerr << "Voxelization failed." << std::endl;
        return;
    }
    
    voxelizedGeometry_ = std::make_unique<VolumetricGeometry>(std::move(grid));
    
    // Now, we take the original geometry from the object and give it our new one.
    originalGeometry_ = obj->setGeometry(std::move(voxelizedGeometry_));

    // After swapping geometry, invalidate caches.
    obj->invalidateMeshCache();
    scene_->MarkStaticGeometryDirty();
}


void VoxelizeCommand::Undo() {
    if (!scene_ || !originalGeometry_) return;
    SceneObject* obj = scene_->get_object_by_id(objectId_);
    if (!obj) return;

    // Swap the original geometry back, and store the voxelized one for redo.
    voxelizedGeometry_ = obj->setGeometry(std::move(originalGeometry_));
    
    obj->invalidateMeshCache();
    scene_->MarkStaticGeometryDirty();
}


} // namespace Urbaxio::Engine

