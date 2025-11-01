#include "engine/engine.h"       // Наш API
#include "engine/scene.h"        // Класс сцены
#include "engine/scene_object.h" // Класс объекта
#include <cad_kernel/cad_kernel.h> // Обертка над OCCT

#include <fmt/core.h>            // Форматирование строк
#include <memory>                // Для std::unique_ptr
#include <iostream>              // Для std::cerr
#include <string>                // Для std::string
#include <vector>                // For std::vector
#include <cmath>                 // For sqrt
#include <map>                   // For subdivision map
#include <glm/glm.hpp>           // For glm::normalize
// #include <stdlib.h> // Больше не нужен для _putenv_s

// OpenCascade includes for capsule generation
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <TopoDS_Shape.hxx>
#include <BRepBuilderAPI_Transform.hxx>

// Глобальный указатель на сцену
std::unique_ptr<Urbaxio::Engine::Scene> g_scene = nullptr;

// --- Определение версии ---
#ifndef URBAXIO_VERSION_STRING
#define URBAXIO_VERSION_STRING "0.0.1-dev"
#endif

namespace { // Anonymous namespace

    // Helper function for icosphere subdivision
    int get_middle_point(int p1, int p2, std::map<long long, int>& middlePointIndexCache, std::vector<glm::vec3>& vertices, float radius) {
        bool firstIsSmaller = p1 < p2;
        long long smallerIndex = firstIsSmaller ? p1 : p2;
        long long greaterIndex = firstIsSmaller ? p2 : p1;
        long long key = (smallerIndex << 32) + greaterIndex;

        auto it = middlePointIndexCache.find(key);
        if (it != middlePointIndexCache.end()) {
            return it->second;
        }

        glm::vec3 point1 = vertices[p1];
        glm::vec3 point2 = vertices[p2];
        glm::vec3 middle = glm::vec3(
            (point1.x + point2.x) / 2.0f,
            (point1.y + point2.y) / 2.0f,
            (point1.z + point2.z) / 2.0f
        );
        
        vertices.push_back(glm::normalize(middle) * radius);
        int i = vertices.size() - 1;
        middlePointIndexCache[key] = i;
        return i;
    }

    // Creates an icosphere mesh with a specified subdivision level
    Urbaxio::CadKernel::MeshBuffers CreateIcoSphereMesh(float radius, int subdivision) {
        Urbaxio::CadKernel::MeshBuffers mesh;
        const float t = (1.0f + std::sqrt(5.0f)) / 2.0f;

        std::vector<glm::vec3> base_vertices = {
            {-1,  t,  0}, { 1,  t,  0}, {-1, -t,  0}, { 1, -t,  0},
            { 0, -1,  t}, { 0,  1,  t}, { 0, -1, -t}, { 0,  1, -t},
            { t,  0, -1}, { t,  0,  1}, {-t,  0, -1}, {-t,  0,  1}
        };

        for (auto& v : base_vertices) {
            v = glm::normalize(v) * radius;
        }

        std::vector<unsigned int> base_indices = {
             0, 11,  5,    0,  5,  1,    0,  1,  7,    0,  7, 10,    0, 10, 11,
             1,  5,  9,    5, 11,  4,   11, 10,  2,   10,  7,  6,    7,  1,  8,
             3,  9,  4,    3,  4,  2,    3,  2,  6,    3,  6,  8,    3,  8,  9,
             4,  9,  5,    2,  4, 11,    6,  2, 10,    8,  6,  7,    9,  8,  1
        };

        std::map<long long, int> middlePointIndexCache;
        std::vector<glm::vec3> temp_vertices_glm = base_vertices;

        for (int s = 0; s < subdivision; s++) {
            std::vector<unsigned int> new_indices;
            for (size_t i = 0; i < base_indices.size(); i += 3) {
                int i0 = base_indices[i];
                int i1 = base_indices[i+1];
                int i2 = base_indices[i+2];
                int a = get_middle_point(i0, i1, middlePointIndexCache, temp_vertices_glm, radius);
                int b = get_middle_point(i1, i2, middlePointIndexCache, temp_vertices_glm, radius);
                int c = get_middle_point(i2, i0, middlePointIndexCache, temp_vertices_glm, radius);
                new_indices.insert(new_indices.end(), { (unsigned int)i0, (unsigned int)a, (unsigned int)c });
                new_indices.insert(new_indices.end(), { (unsigned int)i1, (unsigned int)b, (unsigned int)a });
                new_indices.insert(new_indices.end(), { (unsigned int)i2, (unsigned int)c, (unsigned int)b });
                new_indices.insert(new_indices.end(), { (unsigned int)a,  (unsigned int)b, (unsigned int)c });
            }
            base_indices = new_indices;
        }

        mesh.indices = base_indices;

        mesh.vertices.clear();
        mesh.normals.clear();
        for(const auto& v : temp_vertices_glm) {
            glm::vec3 norm = glm::normalize(v);
            mesh.vertices.push_back(v.x);
            mesh.vertices.push_back(v.y);
            mesh.vertices.push_back(v.z);
            mesh.normals.push_back(norm.x);
            mesh.normals.push_back(norm.y);
            mesh.normals.push_back(norm.z);
        }
        
        return mesh;
    }

    // Creates a capsule mesh along the Z axis
    Urbaxio::CadKernel::MeshBuffers CreateCapsuleMesh(float radius, float height) {
        try {
            // ============ АВТОМАТИЧЕСКИЕ СМЕЩЕНИЯ (зависят от размеров) ============
            // Базовые коэффициенты для правильного отображения:
            float cylinder_z_factor = -0.25f;  // Цилиндр смещается вниз на 25% от высоты
            float sphere_z_factor = 0.25f;     // Сферы смещаются вверх на 25% от высоты
            
            // Вычисляем автоматические смещения:
            float cylinder_offset_x = 0.0f;
            float cylinder_offset_y = 0.0f;
            float cylinder_offset_z = height * cylinder_z_factor;  // Зависит от высоты
            
            float sphere_offset_x = 0.0f;
            float sphere_offset_y = 0.0f;
            float sphere_offset_z = height * sphere_z_factor;      // Зависит от высоты
            // ===================================================================
            
            gp_Pnt center(cylinder_offset_x, cylinder_offset_y, cylinder_offset_z);  // <-- ЦЕНТР ЦИЛИНДРА
            gp_Dir z_dir(0, 0, 1);
            gp_Ax2 axis(center, z_dir);
            
            TopoDS_Shape cylinder = BRepPrimAPI_MakeCylinder(axis, radius, height);

            // Create a single sphere at the origin
            TopoDS_Shape sphere = BRepPrimAPI_MakeSphere(gp_Pnt(0,0,0), radius);

            // Create two transformations to move the sphere to the ends (независимые смещения)
            gp_Trsf top_trsf, bot_trsf;
            top_trsf.SetTranslation(gp_Vec(
                cylinder_offset_x + sphere_offset_x, 
                cylinder_offset_y + sphere_offset_y, 
                cylinder_offset_z + sphere_offset_z + height / 2.0
            ));
            bot_trsf.SetTranslation(gp_Vec(
                cylinder_offset_x + sphere_offset_x, 
                cylinder_offset_y + sphere_offset_y, 
                cylinder_offset_z + sphere_offset_z - height / 2.0
            ));

            // Apply transformations
            TopoDS_Shape top_hemisphere = BRepBuilderAPI_Transform(sphere, top_trsf);
            TopoDS_Shape bot_hemisphere = BRepBuilderAPI_Transform(sphere, bot_trsf);
            
            // Fuse them
            BRepAlgoAPI_Fuse fuser(cylinder, top_hemisphere);
            fuser.Build();
            TopoDS_Shape result = fuser.Shape();
            
            BRepAlgoAPI_Fuse final_fuser(result, bot_hemisphere);
            final_fuser.Build();
            result = final_fuser.Shape();
            
            return Urbaxio::CadKernel::TriangulateShape(result);

        } catch(...) {
            std::cerr << "OCCT Exception during capsule creation!" << std::endl;
            return Urbaxio::CadKernel::MeshBuffers();
        }
    }
} // end anonymous namespace

// --- Реализация функций API ---
#ifdef __cplusplus
extern "C" {
#endif

    URBAXIO_API void initialize_engine() {
        fmt::print("Engine: Initializing Urbaxio Engine v{}...\n", URBAXIO_VERSION_STRING);

        // Инициализируем CAD ядро (обертку)
        Urbaxio::CadKernel::initialize(); // Вызываем инициализацию OCCT

        if (!g_scene) {
            g_scene = std::make_unique<Urbaxio::Engine::Scene>();
            fmt::print("Engine: Scene created successfully.\n");

        }
        else {
            fmt::print("Engine: Engine already initialized.\n");
        }
    }

    // Функция для получения доступа к глобальной сцене из C API
    URBAXIO_API Scene* get_engine_scene() {
        return reinterpret_cast<Scene*>(g_scene.get());
    }

#ifdef __cplusplus
} // extern "C"
#endif 