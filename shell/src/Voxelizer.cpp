#include "Voxelizer.h"
#include "engine/geometry/VoxelGrid.h"

// All heavy includes are now isolated in this file
#include <cad_kernel/cad_kernel.h>
#include <igl/signed_distance.h>
#include <Eigen/Core>
#include <TopoDS_Shape.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <ShapeFix_Shape.hxx>
#include <iostream>
#include <cstring>

namespace Urbaxio {

std::unique_ptr<Engine::VoxelGrid> Voxelizer::BRepToSDFGrid(const TopoDS_Shape& shape, int resolution) {
    if (shape.IsNull()) {
        return nullptr;
    }

    // --- STEP 1: Heal the B-Rep shape before triangulation ---
    Handle(ShapeFix_Shape) fixer = new ShapeFix_Shape(shape);
    fixer->Perform();
    TopoDS_Shape healedShape = fixer->Shape();

    // 2. Tessellate the B-Rep shape into a triangle mesh
    CadKernel::MeshBuffers mesh = CadKernel::TriangulateShape(healedShape);
    if (mesh.isEmpty()) {
        std::cerr << "Voxelize Error: Failed to triangulate B-Rep shape." << std::endl;
        return nullptr;
    }

    // 3. Convert mesh data to Eigen matrices for libigl
    Eigen::MatrixXd V(mesh.vertices.size() / 3, 3);
    for (size_t i = 0; i < V.rows(); ++i) {
        V(i, 0) = mesh.vertices[i * 3 + 0];
        V(i, 1) = mesh.vertices[i * 3 + 1];
        V(i, 2) = mesh.vertices[i * 3 + 2];
    }

    Eigen::MatrixXi F(mesh.indices.size() / 3, 3);
    for (size_t i = 0; i < F.rows(); ++i) {
        F(i, 0) = mesh.indices[i * 3 + 0];
        F(i, 1) = mesh.indices[i * 3 + 1];
        F(i, 2) = mesh.indices[i * 3 + 2];
    }

    // 4. Define the voxel grid dimensions based on the mesh bounding box
    Eigen::Vector3d box_min = V.colwise().minCoeff();
    Eigen::Vector3d box_max = V.colwise().maxCoeff();
    Eigen::Vector3d size = box_max - box_min;
    double padding = size.maxCoeff() * 0.1;
    box_min.array() -= padding;
    box_max.array() += padding;
    size = box_max - box_min;

    double voxelSize = size.maxCoeff() / (double)resolution;
    glm::uvec3 dims(ceil(size.x() / voxelSize), ceil(size.y() / voxelSize), ceil(size.z() / voxelSize));
    glm::vec3 origin(box_min.x(), box_min.y(), box_min.z());

    // 5. Create the grid of query points for libigl
    Eigen::MatrixXd P(dims.x * dims.y * dims.z, 3);
    for (unsigned int z = 0; z < dims.z; ++z) for (unsigned int y = 0; y < dims.y; ++y) for (unsigned int x = 0; x < dims.x; ++x) {
        size_t index = z * dims.x * dims.y + y * dims.x + x;
        P(index, 0) = origin.x + (x + 0.5) * voxelSize;
        P(index, 1) = origin.y + (y + 0.5) * voxelSize;
        P(index, 2) = origin.z + (z + 0.5) * voxelSize;
    }
    
    // 6. Call libigl to compute the SDF for all points at once
    Eigen::VectorXd S; 
    Eigen::VectorXi I; 
    Eigen::MatrixXd C; 
    Eigen::MatrixXd N; 
    igl::signed_distance(P, V, F, igl::SIGNED_DISTANCE_TYPE_WINDING_NUMBER, S, I, C, N);

    // 7. Copy the results into our sparse OpenVDB VoxelGrid structure
    auto grid = std::make_unique<Engine::VoxelGrid>(dims, origin, (float)voxelSize);
    
    // Use OpenVDB accessor for efficient batch writes
    Engine::VoxelGrid::Accessor accessor(*grid);
    
    size_t activeCount = 0;
    for (unsigned int z = 0; z < dims.z; ++z) {
        for (unsigned int y = 0; y < dims.y; ++y) {
            for (unsigned int x = 0; x < dims.x; ++x) {
                size_t index = z * dims.x * dims.y + y * dims.x + x;
                float sdfValue = static_cast<float>(S(index));
                
                // Only store values near the surface (sparse optimization)
                // This dramatically reduces memory usage
                if (std::abs(sdfValue) < voxelSize * 5.0f) {
                    accessor.setValue(x, y, z, sdfValue);
                    activeCount++;
                }
            }
        }
    }
    
    std::cout << "Voxelization complete. Grid size: " << dims.x << "x" << dims.y << "x" << dims.z 
              << ", Active voxels: " << activeCount << " / " << (dims.x * dims.y * dims.z)
              << " (" << (100.0 * activeCount / (dims.x * dims.y * dims.z)) << "%)"
              << ", Memory: " << (grid->getMemoryUsage() / 1024.0 / 1024.0) << " MB" << std::endl;
    return grid;
}

} // namespace Urbaxio

