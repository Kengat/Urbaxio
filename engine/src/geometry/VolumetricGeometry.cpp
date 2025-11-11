#include "engine/geometry/VolumetricGeometry.h"

#include "engine/geometry/VoxelGrid.h"
#include "cad_kernel/MeshBuffers.h"
#include <openvdb/tools/VolumeToMesh.h>
#include <igl/per_vertex_normals.h>
#include <Eigen/Core>

#include <iostream>

namespace Urbaxio::Engine {

VolumetricGeometry::VolumetricGeometry(std::unique_ptr<VoxelGrid> grid)
    : grid_(std::move(grid)) {}

VolumetricGeometry::~VolumetricGeometry() = default;

VoxelGrid* VolumetricGeometry::getGrid() {
    return grid_.get();
}

const VoxelGrid* VolumetricGeometry::getGrid() const {
    return grid_.get();
}

CadKernel::MeshBuffers VolumetricGeometry::getRenderMesh(double detailLevel) const {
    CadKernel::MeshBuffers outMesh;
    if (!grid_ || !grid_->grid_) {
        return outMesh;
    }

    std::cout << "[VolumetricGeometry] OpenVDB stats: " 
              << grid_->getActiveVoxelCount() << " active voxels, "
              << (grid_->getMemoryUsage() / 1024.0 / 1024.0) << " MB" << std::endl;

    // Use OpenVDB's native volumeToMesh - works directly on sparse grid!
    // This is MUCH faster and uses less memory than converting to dense array
    std::vector<openvdb::Vec3s> points;
    std::vector<openvdb::Vec3I> triangles;
    std::vector<openvdb::Vec4I> quads;
    
    const double isoValue = 0.0;
    const double adaptivity = 0.0; // 0.0 = uniform (like Marching Cubes)
    
    try {
        openvdb::tools::volumeToMesh(
            *grid_->grid_, 
            points, 
            triangles, 
            quads, 
            isoValue, 
            adaptivity
        );
        
        if (!points.empty() && (!triangles.empty() || !quads.empty())) {
            // Convert points
            outMesh.vertices.reserve(points.size() * 3);
            for (const openvdb::Vec3s& p : points) {
                outMesh.vertices.push_back(p.x());
                outMesh.vertices.push_back(p.y());
                outMesh.vertices.push_back(p.z());
            }
            
            // Convert triangles and quads (fix winding order for correct normals)
            outMesh.indices.reserve(triangles.size() * 3 + quads.size() * 6);
            
            for (const openvdb::Vec3I& t : triangles) {
                outMesh.indices.push_back(t.x());
                outMesh.indices.push_back(t.z()); 
                outMesh.indices.push_back(t.y()); 
            }
            
            for (const openvdb::Vec4I& q : quads) {
                // Triangulate quad with correct winding
                outMesh.indices.push_back(q.x());
                outMesh.indices.push_back(q.z()); 
                outMesh.indices.push_back(q.y()); 
                
                outMesh.indices.push_back(q.x());
                outMesh.indices.push_back(q.w()); 
                outMesh.indices.push_back(q.z()); 
            }
            
            // Calculate normals using libigl
            if (!outMesh.indices.empty()) {
                Eigen::MatrixXd V_igl(points.size(), 3);
                for(size_t i = 0; i < points.size(); ++i) {
                    V_igl(i, 0) = points[i].x();
                    V_igl(i, 1) = points[i].y();
                    V_igl(i, 2) = points[i].z();
                }
                
                Eigen::MatrixXi F_igl(outMesh.indices.size() / 3, 3);
                for (size_t i = 0; i < F_igl.rows(); ++i) {
                    F_igl(i, 0) = outMesh.indices[i * 3 + 0];
                    F_igl(i, 1) = outMesh.indices[i * 3 + 1];
                    F_igl(i, 2) = outMesh.indices[i * 3 + 2];
                }
                
                Eigen::MatrixXd N_igl;
                igl::per_vertex_normals(V_igl, F_igl, N_igl);
                
                outMesh.normals.resize(N_igl.rows() * 3);
                for (int i = 0; i < N_igl.rows(); ++i) {
                    outMesh.normals[i * 3 + 0] = (float)N_igl(i, 0);
                    outMesh.normals[i * 3 + 1] = (float)N_igl(i, 1);
                    outMesh.normals[i * 3 + 2] = (float)N_igl(i, 2);
                }
                
                // Placeholder UVs
                outMesh.uvs.assign(points.size() * 2, 0.0f);
                
                std::cout << "[VolumetricGeometry] ✅ Mesh generated: " 
                          << points.size() << " vertices, " 
                          << (outMesh.indices.size() / 3) << " triangles" << std::endl;
            }
        } else {
            std::cout << "[VolumetricGeometry] No surface generated (empty grid or no zero crossing)" << std::endl;
        }
    } catch (const openvdb::Exception& e) {
        std::cerr << "[VolumetricGeometry] ❌ OpenVDB volumeToMesh failed: " << e.what() << std::endl;
    }

    return outMesh;
}

} // namespace Urbaxio::Engine

