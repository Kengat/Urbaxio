#include "engine/engine.h"       // Наш API
#include "engine/scene.h"        // Класс сцены
#include "engine/scene_object.h" // Класс объекта
#include <cad_kernel/cad_kernel.h> // Обертка над OCCT

#include <fmt/core.h>            // Форматирование строк
#include <memory>                // Для std::unique_ptr
#include <iostream>              // Для std::cerr
#include <string>                // Для std::string
// #include <stdlib.h> // Больше не нужен для _putenv_s

// Глобальный указатель на сцену
std::unique_ptr<Urbaxio::Engine::Scene> g_scene = nullptr;

// --- Определение версии ---
#ifndef URBAXIO_VERSION_STRING
#define URBAXIO_VERSION_STRING "0.0.1-dev"
#endif

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

            // Создаем тестовые объекты при старте
            auto* obj1 = g_scene->create_object("MyFirstObject");
            auto* obj2 = g_scene->create_object("AnotherObject");

            if (obj1 && obj2) {
                fmt::print("Engine: Created initial test object '{}' with ID {}.\n", obj1->get_name(), obj1->get_id());
                fmt::print("Engine: Created initial test object '{}' with ID {}.\n", obj2->get_name(), obj2->get_id());
            }
            else {
                fmt::print(stderr, "Engine: Error creating initial test objects!\n");
            }

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