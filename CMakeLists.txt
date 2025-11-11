# Минимальная версия CMake
cmake_minimum_required(VERSION 3.18)

# Название проекта
project(Urbaxio VERSION 0.0.1 LANGUAGES CXX)

# --- ГЛОБАЛЬНОЕ ДОБАВЛЕНИЕ ПУТИ ---
include_directories("C:/vcpkg/installed/x64-windows/include")

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
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:/utf-8>)
endif()

# Включаем папку cmake для поиска модулей
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Добавляем подпапку с CAD ядром (оберткой)
add_subdirectory(cad_kernel)

# Добавляем подпапку с движком
add_subdirectory(engine)

# --- GPU Acceleration (Optional) ---
# Check if CUDA is available and enable GPU support
option(ENABLE_GPU "Enable GPU acceleration with CUDA" ON)

if(ENABLE_GPU)
    include(CheckLanguage)
    check_language(CUDA)
    
    if(CMAKE_CUDA_COMPILER)
        enable_language(CUDA)
        message(STATUS "CUDA found: ${CMAKE_CUDA_COMPILER}")
        message(STATUS "CUDA Version: ${CMAKE_CUDA_COMPILER_VERSION}")
        
        # CUDA-specific settings to avoid MSBuild issues
        set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --use-local-env")
        set(CMAKE_CUDA_STANDARD 17)
        set(CMAKE_CUDA_STANDARD_REQUIRED ON)
        
        # Add GPU module
        add_subdirectory(engine/src/gpu)
        
        # Mark GPU as enabled for the rest of the project
        add_compile_definitions(URBAXIO_GPU_ENABLED=1)
        message(STATUS "GPU acceleration ENABLED")
    else()
        message(WARNING "CUDA not found. GPU acceleration will be DISABLED.")
        message(WARNING "To enable GPU support, install CUDA Toolkit from:")
        message(WARNING "  https://developer.nvidia.com/cuda-downloads")
        add_compile_definitions(URBAXIO_GPU_ENABLED=0)
    endif()
else()
    message(STATUS "GPU acceleration DISABLED (ENABLE_GPU=OFF)")
    add_compile_definitions(URBAXIO_GPU_ENABLED=0)
endif()

# Добавляем подпапку с исполняемым файлом приложения
add_subdirectory(shell)

# --- Пользовательские цели для генерации ---

# 1. Генерация ПОЛНОГО снапшота кода (старое поведение)
add_custom_target(generate_snapshot
    COMMAND ${CMAKE_COMMAND} -DPROJECT_SOURCE_DIR=${CMAKE_SOURCE_DIR} -DFORCE_FULL_SNAPSHOT=TRUE -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/GenerateSnapshot.cmake"
    COMMENT "Generating FULL project_snapshot.txt..."
    VERBATIM
)
message(STATUS "Added custom target 'generate_snapshot'.")

# 2. Генерация структуры проекта
add_custom_target(generate_structure
    COMMAND ${CMAKE_COMMAND} -DPROJECT_SOURCE_DIR=${CMAKE_SOURCE_DIR} -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/GenerateStructure.cmake"
    COMMENT "Generating project_structure.txt with file list..."
    VERBATIM
)
message(STATUS "Added custom target 'generate_structure'.")

# 3. Генерация ЧАСТИЧНОГО снапшота (читает из snapshot_files.txt)
add_custom_target(generate_partial_snapshot
    COMMAND ${CMAKE_COMMAND} -DPROJECT_SOURCE_DIR=${CMAKE_SOURCE_DIR} -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/GenerateSnapshot.cmake"
    COMMENT "Generating partial_snapshot.txt from snapshot_files.txt..."
    VERBATIM
)
message(STATUS "Added custom target 'generate_partial_snapshot'.")


# --- Настройки для удобства разработки ---
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# --- Выводим информационные сообщения ---
message(STATUS "Project Root: ${CMAKE_SOURCE_DIR}")
message(STATUS "Build Directory: ${CMAKE_BINARY_DIR}")
message(STATUS "Using C++ Standard: ${CMAKE_CXX_STANDARD}") 