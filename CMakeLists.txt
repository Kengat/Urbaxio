# Минимальная версия CMake
cmake_minimum_required(VERSION 3.18)

# Название проекта
project(Urbaxio VERSION 0.0.1 LANGUAGES CXX)

# --- Find OpenCASCADE (MANUAL, HARDCODED PATH) ---
# We are completely bypassing vcpkg's manifest for OpenCASCADE.
# We point directly to the files installed on your system.
set(OCCT_INSTALL_DIR "C:/vcpkg/installed/x64-windows")
message(STATUS "Using hardcoded OpenCASCADE path: ${OCCT_INSTALL_DIR}")

set(OpenCASCADE_INCLUDE_DIR "${OCCT_INSTALL_DIR}/include/opencascade")
include_directories(${OpenCASCADE_INCLUDE_DIR})
message(STATUS "Manually set OpenCASCADE include dir: ${OpenCASCADE_INCLUDE_DIR}")

set(OCCT_LIB_DIR_RELEASE "${OCCT_INSTALL_DIR}/lib")
set(OCCT_LIB_DIR_DEBUG "${OCCT_INSTALL_DIR}/debug/lib")

# List of required modules for OCCT 7.9.1. TKBOP is now TKBool.
# --- ИЗМЕНИТЬ ЭТУ СТРОКУ ---
set(OCCT_MODULES TKernel TKMath TKPrim TKBRep TKTopAlgo TKG2d TKG3d TKMesh TKBO TKService TKShHealing TKBool TKOffset)
set(OpenCASCADE_LIBRARIES "")
foreach(module ${OCCT_MODULES})
    list(APPEND OpenCASCADE_LIBRARIES "optimized" "${OCCT_LIB_DIR_RELEASE}/${module}.lib" "debug" "${OCCT_LIB_DIR_DEBUG}/${module}.lib")
endforeach()
message(STATUS "Manually located OpenCASCADE libraries from hardcoded path.")
# --- End OpenCASCADE Find ---


# Определяем версию для использования в коде
set(PROJECT_VERSION ${Urbaxio_VERSION})
add_compile_definitions(URBAXIO_VERSION_STRING="${PROJECT_VERSION}")
message(STATUS "Project version: ${PROJECT_VERSION}")

# Устанавливаем стандарт C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add compile option for UTF-8 source files, required by libraries like fmt
if(MSVC)
    add_compile_options(/utf-8)
endif()

# Включаем папку cmake для поиска модулей
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Добавляем подпапку с CAD ядром (оберткой)
add_subdirectory(cad_kernel)

# Добавляем подпапку с движком
add_subdirectory(engine)

# Добавляем подпапку с исполняемым файлом приложения
add_subdirectory(shell)

# --- Пользовательская цель для генерации снапшота кода ---
add_custom_target(generate_snapshot
    # Запускаем CMake в режиме скрипта для выполнения GenerateSnapshot.cmake
    COMMAND ${CMAKE_COMMAND} -DPROJECT_SOURCE_DIR=${CMAKE_SOURCE_DIR} -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/GenerateSnapshot.cmake"
    COMMENT "Generating project_snapshot.txt with all source code..."
    VERBATIM # Важно для правильной передачи команд
)
message(STATUS "Added custom target 'generate_snapshot'. Run 'cmake --build . --target generate_snapshot' to execute.")


# --- Настройки для удобства разработки ---
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# --- Выводим информационные сообщения ---
message(STATUS "Project Root: ${CMAKE_SOURCE_DIR}")
message(STATUS "Build Directory: ${CMAKE_BINARY_DIR}")
message(STATUS "Using C++ Standard: ${CMAKE_CXX_STANDARD}") 