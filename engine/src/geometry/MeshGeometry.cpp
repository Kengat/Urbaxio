#include "engine/geometry/MeshGeometry.h"
#include <utility>
#include <algorithm>
#include <cmath>
#include <map>
#include <cstring>

namespace Urbaxio::Engine {

    MeshGeometry::MeshGeometry(CadKernel::MeshBuffers mesh) : mesh_(std::move(mesh)) {}

    MeshGeometry::~MeshGeometry() = default;

    CadKernel::MeshBuffers MeshGeometry::getRenderMesh(double detailLevel) const {
        // Just return a copy of the stored mesh. detailLevel is ignored for this geometry type.
        return mesh_;
    }

    const CadKernel::MeshBuffers& MeshGeometry::getMesh() const {
        return mesh_;
    }

    CadKernel::MeshBuffers& MeshGeometry::getMutableMesh() {
        return mesh_;
    }

    bool MeshGeometry::moveVertices(const std::set<unsigned int>& vertexIndices, const glm::vec3& translation) {
        if (vertexIndices.empty()) return false;

        for (unsigned int idx : vertexIndices) {
            size_t base = static_cast<size_t>(idx) * 3;
            if (base + 2 < mesh_.vertices.size()) {
                mesh_.vertices[base + 0] += translation.x;
                mesh_.vertices[base + 1] += translation.y;
                mesh_.vertices[base + 2] += translation.z;
            }
        }
        return true;
    }

    unsigned int MeshGeometry::addVertex(const glm::vec3& pos, const glm::vec3& normal) {
        unsigned int newIndex = static_cast<unsigned int>(mesh_.vertices.size() / 3);
        mesh_.vertices.push_back(pos.x);
        mesh_.vertices.push_back(pos.y);
        mesh_.vertices.push_back(pos.z);
        mesh_.normals.push_back(normal.x);
        mesh_.normals.push_back(normal.y);
        mesh_.normals.push_back(normal.z);
        mesh_.uvs.push_back(0.0f);
        mesh_.uvs.push_back(0.0f);
        return newIndex;
    }

    void MeshGeometry::addTriangle(unsigned int i0, unsigned int i1, unsigned int i2) {
        mesh_.indices.push_back(i0);
        mesh_.indices.push_back(i1);
        mesh_.indices.push_back(i2);
    }

    std::vector<std::pair<unsigned int, unsigned int>> MeshGeometry::findBoundaryEdges(
        const std::set<unsigned int>& faceVertexIndices) const {

        // Count how many times each edge appears in triangles that are part of the face
        std::map<std::pair<unsigned int, unsigned int>, int> edgeCount;

        size_t numTriangles = mesh_.indices.size() / 3;
        for (size_t t = 0; t < numTriangles; ++t) {
            unsigned int i0 = mesh_.indices[t * 3 + 0];
            unsigned int i1 = mesh_.indices[t * 3 + 1];
            unsigned int i2 = mesh_.indices[t * 3 + 2];

            // Check if all vertices of this triangle are in the face
            bool inFace = (faceVertexIndices.count(i0) > 0) &&
                          (faceVertexIndices.count(i1) > 0) &&
                          (faceVertexIndices.count(i2) > 0);
            if (!inFace) continue;

            // Add edges (ordered so smaller index first for consistent keying)
            auto makeEdge = [](unsigned int a, unsigned int b) {
                return a < b ? std::make_pair(a, b) : std::make_pair(b, a);
            };

            edgeCount[makeEdge(i0, i1)]++;
            edgeCount[makeEdge(i1, i2)]++;
            edgeCount[makeEdge(i2, i0)]++;
        }

        // Boundary edges are those that appear exactly once
        std::vector<std::pair<unsigned int, unsigned int>> boundary;
        for (const auto& [edge, count] : edgeCount) {
            if (count == 1) {
                boundary.push_back(edge);
            }
        }

        return boundary;
    }

    bool MeshGeometry::extrudeFace(const std::set<unsigned int>& faceVertexIndices,
                                    const glm::vec3& normal, float distance) {
        if (faceVertexIndices.empty() || std::abs(distance) < 1e-6f) return false;

        glm::vec3 faceNormal = glm::normalize(normal);
        glm::vec3 offset = faceNormal * distance;

        // Find boundary edges of the selected face
        auto boundaryEdges = findBoundaryEdges(faceVertexIndices);
        if (boundaryEdges.empty()) return false;

        // Create mapping from old vertex indices to new (offset) vertex indices
        // Store new vertices with placeholder normals (will recalculate later)
        std::map<unsigned int, unsigned int> oldToNew;
        for (unsigned int oldIdx : faceVertexIndices) {
            size_t base = static_cast<size_t>(oldIdx) * 3;
            if (base + 2 >= mesh_.vertices.size()) continue;

            glm::vec3 pos(mesh_.vertices[base], mesh_.vertices[base + 1], mesh_.vertices[base + 2]);
            glm::vec3 newPos = pos + offset;
            // Use face normal for top face vertices (will be corrected by recalculateNormals)
            unsigned int newIdx = addVertex(newPos, faceNormal);
            oldToNew[oldIdx] = newIdx;
        }

        // Find all triangles that use only face vertices and update them to use new vertices
        // (This moves the "top" face to the new position)
        size_t numTrianglesBeforeSides = mesh_.indices.size() / 3;
        for (size_t t = 0; t < numTrianglesBeforeSides; ++t) {
            unsigned int& i0 = mesh_.indices[t * 3 + 0];
            unsigned int& i1 = mesh_.indices[t * 3 + 1];
            unsigned int& i2 = mesh_.indices[t * 3 + 2];

            bool allInFace = (faceVertexIndices.count(i0) > 0) &&
                             (faceVertexIndices.count(i1) > 0) &&
                             (faceVertexIndices.count(i2) > 0);

            if (allInFace) {
                // Replace with new vertices
                auto it0 = oldToNew.find(i0);
                auto it1 = oldToNew.find(i1);
                auto it2 = oldToNew.find(i2);
                if (it0 != oldToNew.end()) i0 = it0->second;
                if (it1 != oldToNew.end()) i1 = it1->second;
                if (it2 != oldToNew.end()) i2 = it2->second;
            }
        }

        // Calculate face center for winding determination
        glm::vec3 faceCenter(0.0f);
        for (unsigned int idx : faceVertexIndices) {
            size_t base = static_cast<size_t>(idx) * 3;
            faceCenter += glm::vec3(mesh_.vertices[base], mesh_.vertices[base + 1], mesh_.vertices[base + 2]);
        }
        faceCenter /= static_cast<float>(faceVertexIndices.size());

        // Create side faces (quads as 2 triangles each) for each boundary edge
        for (const auto& [ev1, ev2] : boundaryEdges) {
            auto it1 = oldToNew.find(ev1);
            auto it2 = oldToNew.find(ev2);
            if (it1 == oldToNew.end() || it2 == oldToNew.end()) continue;

            unsigned int nv1 = it1->second;
            unsigned int nv2 = it2->second;

            // Get vertex positions
            glm::vec3 p1(mesh_.vertices[ev1 * 3], mesh_.vertices[ev1 * 3 + 1], mesh_.vertices[ev1 * 3 + 2]);
            glm::vec3 p2(mesh_.vertices[ev2 * 3], mesh_.vertices[ev2 * 3 + 1], mesh_.vertices[ev2 * 3 + 2]);
            glm::vec3 np1(mesh_.vertices[nv1 * 3], mesh_.vertices[nv1 * 3 + 1], mesh_.vertices[nv1 * 3 + 2]);
            glm::vec3 np2(mesh_.vertices[nv2 * 3], mesh_.vertices[nv2 * 3 + 1], mesh_.vertices[nv2 * 3 + 2]);

            // Edge direction and midpoint
            glm::vec3 edgeDir = p2 - p1;
            glm::vec3 edgeMid = (p1 + p2) * 0.5f;

            // Calculate outward direction (from face center to edge midpoint)
            glm::vec3 outwardDir = glm::normalize(edgeMid - faceCenter);

            // Side face normal should point outward
            // Use cross product of edge direction and extrusion direction
            glm::vec3 sideNormal = glm::normalize(glm::cross(edgeDir, offset));

            // Flip if pointing inward
            if (glm::dot(sideNormal, outwardDir) < 0.0f) {
                sideNormal = -sideNormal;
            }

            // Create 4 new vertices for the side quad with correct normals
            unsigned int sv1 = addVertex(p1, sideNormal);
            unsigned int sv2 = addVertex(p2, sideNormal);
            unsigned int sv3 = addVertex(np2, sideNormal);
            unsigned int sv4 = addVertex(np1, sideNormal);

            // Determine winding: we want CCW when viewed from outside (along sideNormal)
            // Quad vertices: sv1(p1) -> sv2(p2) -> sv3(np2) -> sv4(np1)
            // If extrusion is upward, the quad goes: bottom-left, bottom-right, top-right, top-left
            // Two triangles: (sv1, sv2, sv3) and (sv1, sv3, sv4)

            // Check if this winding gives correct normal direction
            glm::vec3 triNormal = glm::cross(
                glm::vec3(mesh_.vertices[sv2 * 3], mesh_.vertices[sv2 * 3 + 1], mesh_.vertices[sv2 * 3 + 2]) -
                glm::vec3(mesh_.vertices[sv1 * 3], mesh_.vertices[sv1 * 3 + 1], mesh_.vertices[sv1 * 3 + 2]),
                glm::vec3(mesh_.vertices[sv3 * 3], mesh_.vertices[sv3 * 3 + 1], mesh_.vertices[sv3 * 3 + 2]) -
                glm::vec3(mesh_.vertices[sv1 * 3], mesh_.vertices[sv1 * 3 + 1], mesh_.vertices[sv1 * 3 + 2])
            );

            if (glm::dot(triNormal, sideNormal) > 0.0f) {
                // Correct winding
                addTriangle(sv1, sv2, sv3);
                addTriangle(sv1, sv3, sv4);
            } else {
                // Reverse winding
                addTriangle(sv1, sv3, sv2);
                addTriangle(sv1, sv4, sv3);
            }
        }

        return true;
    }

    std::unique_ptr<MeshGeometry> MeshGeometry::createFromVertexLoop(
        const std::vector<glm::vec3>& vertices, const glm::vec3& normal) {

        if (vertices.size() < 3) return nullptr;

        CadKernel::MeshBuffers mesh;
        glm::vec3 faceNormal = glm::normalize(normal);

        // Calculate face normal from vertices to check winding
        glm::vec3 edge1 = vertices[1] - vertices[0];
        glm::vec3 edge2 = vertices[2] - vertices[0];
        glm::vec3 calculatedNormal = glm::cross(edge1, edge2);

        // Determine if we need to reverse winding (if calculated normal is opposite to desired)
        bool reverseWinding = glm::dot(calculatedNormal, faceNormal) < 0.0f;

        // Add vertices for front face
        for (const auto& v : vertices) {
            mesh.vertices.push_back(v.x);
            mesh.vertices.push_back(v.y);
            mesh.vertices.push_back(v.z);
            mesh.normals.push_back(faceNormal.x);
            mesh.normals.push_back(faceNormal.y);
            mesh.normals.push_back(faceNormal.z);
            mesh.uvs.push_back(0.0f);
            mesh.uvs.push_back(0.0f);
        }

        // Add vertices for back face (with reversed normal)
        size_t backFaceStart = vertices.size();
        for (const auto& v : vertices) {
            mesh.vertices.push_back(v.x);
            mesh.vertices.push_back(v.y);
            mesh.vertices.push_back(v.z);
            mesh.normals.push_back(-faceNormal.x);
            mesh.normals.push_back(-faceNormal.y);
            mesh.normals.push_back(-faceNormal.z);
            mesh.uvs.push_back(0.0f);
            mesh.uvs.push_back(0.0f);
        }

        // Fan triangulation for front face
        for (size_t i = 1; i + 1 < vertices.size(); ++i) {
            if (reverseWinding) {
                mesh.indices.push_back(0);
                mesh.indices.push_back(static_cast<unsigned int>(i + 1));
                mesh.indices.push_back(static_cast<unsigned int>(i));
            } else {
                mesh.indices.push_back(0);
                mesh.indices.push_back(static_cast<unsigned int>(i));
                mesh.indices.push_back(static_cast<unsigned int>(i + 1));
            }
        }

        // Fan triangulation for back face (reversed winding)
        for (size_t i = 1; i + 1 < vertices.size(); ++i) {
            if (reverseWinding) {
                mesh.indices.push_back(static_cast<unsigned int>(backFaceStart));
                mesh.indices.push_back(static_cast<unsigned int>(backFaceStart + i));
                mesh.indices.push_back(static_cast<unsigned int>(backFaceStart + i + 1));
            } else {
                mesh.indices.push_back(static_cast<unsigned int>(backFaceStart));
                mesh.indices.push_back(static_cast<unsigned int>(backFaceStart + i + 1));
                mesh.indices.push_back(static_cast<unsigned int>(backFaceStart + i));
            }
        }

        return std::make_unique<MeshGeometry>(std::move(mesh));
    }

    std::vector<char> MeshGeometry::serialize() const {
        std::vector<char> data;

        // Format: [numVertices][vertices...][numNormals][normals...][numUVs][uvs...][numIndices][indices...]
        auto writeSize = [&data](size_t size) {
            uint32_t s = static_cast<uint32_t>(size);
            const char* p = reinterpret_cast<const char*>(&s);
            data.insert(data.end(), p, p + sizeof(uint32_t));
        };

        auto writeFloats = [&data](const std::vector<float>& v) {
            const char* p = reinterpret_cast<const char*>(v.data());
            data.insert(data.end(), p, p + v.size() * sizeof(float));
        };

        auto writeIndices = [&data](const std::vector<unsigned int>& v) {
            const char* p = reinterpret_cast<const char*>(v.data());
            data.insert(data.end(), p, p + v.size() * sizeof(unsigned int));
        };

        writeSize(mesh_.vertices.size());
        writeFloats(mesh_.vertices);

        writeSize(mesh_.normals.size());
        writeFloats(mesh_.normals);

        writeSize(mesh_.uvs.size());
        writeFloats(mesh_.uvs);

        writeSize(mesh_.indices.size());
        writeIndices(mesh_.indices);

        return data;
    }

    std::unique_ptr<MeshGeometry> MeshGeometry::deserialize(const std::vector<char>& data) {
        if (data.empty()) return nullptr;

        size_t offset = 0;

        auto readSize = [&data, &offset]() -> uint32_t {
            if (offset + sizeof(uint32_t) > data.size()) return 0;
            uint32_t s;
            std::memcpy(&s, data.data() + offset, sizeof(uint32_t));
            offset += sizeof(uint32_t);
            return s;
        };

        auto readFloats = [&data, &offset](size_t count) -> std::vector<float> {
            std::vector<float> v(count);
            size_t bytes = count * sizeof(float);
            if (offset + bytes <= data.size()) {
                std::memcpy(v.data(), data.data() + offset, bytes);
                offset += bytes;
            }
            return v;
        };

        auto readIndices = [&data, &offset](size_t count) -> std::vector<unsigned int> {
            std::vector<unsigned int> v(count);
            size_t bytes = count * sizeof(unsigned int);
            if (offset + bytes <= data.size()) {
                std::memcpy(v.data(), data.data() + offset, bytes);
                offset += bytes;
            }
            return v;
        };

        CadKernel::MeshBuffers mesh;

        uint32_t numVertices = readSize();
        mesh.vertices = readFloats(numVertices);

        uint32_t numNormals = readSize();
        mesh.normals = readFloats(numNormals);

        uint32_t numUVs = readSize();
        mesh.uvs = readFloats(numUVs);

        uint32_t numIndices = readSize();
        mesh.indices = readIndices(numIndices);

        return std::make_unique<MeshGeometry>(std::move(mesh));
    }

    void MeshGeometry::recalculateNormals() {
        size_t numVertices = mesh_.vertices.size() / 3;
        if (numVertices == 0) return;

        // Reset normals to zero
        mesh_.normals.assign(numVertices * 3, 0.0f);

        // Accumulate face normals weighted by area
        size_t numTriangles = mesh_.indices.size() / 3;
        for (size_t t = 0; t < numTriangles; ++t) {
            unsigned int i0 = mesh_.indices[t * 3 + 0];
            unsigned int i1 = mesh_.indices[t * 3 + 1];
            unsigned int i2 = mesh_.indices[t * 3 + 2];

            glm::vec3 v0(mesh_.vertices[i0 * 3], mesh_.vertices[i0 * 3 + 1], mesh_.vertices[i0 * 3 + 2]);
            glm::vec3 v1(mesh_.vertices[i1 * 3], mesh_.vertices[i1 * 3 + 1], mesh_.vertices[i1 * 3 + 2]);
            glm::vec3 v2(mesh_.vertices[i2 * 3], mesh_.vertices[i2 * 3 + 1], mesh_.vertices[i2 * 3 + 2]);

            glm::vec3 edge1 = v1 - v0;
            glm::vec3 edge2 = v2 - v0;
            glm::vec3 faceNormal = glm::cross(edge1, edge2); // Not normalized - length = 2*area

            // Add to each vertex
            for (unsigned int idx : {i0, i1, i2}) {
                mesh_.normals[idx * 3 + 0] += faceNormal.x;
                mesh_.normals[idx * 3 + 1] += faceNormal.y;
                mesh_.normals[idx * 3 + 2] += faceNormal.z;
            }
        }

        // Normalize all normals
        for (size_t i = 0; i < numVertices; ++i) {
            glm::vec3 n(mesh_.normals[i * 3], mesh_.normals[i * 3 + 1], mesh_.normals[i * 3 + 2]);
            float len = glm::length(n);
            if (len > 1e-6f) {
                n /= len;
                mesh_.normals[i * 3 + 0] = n.x;
                mesh_.normals[i * 3 + 1] = n.y;
                mesh_.normals[i * 3 + 2] = n.z;
            }
        }
    }

} // namespace Urbaxio::Engine
