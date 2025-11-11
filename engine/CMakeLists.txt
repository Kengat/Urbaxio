# Определяем статическую библиотеку "engine"
add_library(engine STATIC
    src/engine_main.cpp
    src/scene.cpp
    src/scene_object.cpp
    src/geometry/BRepGeometry.cpp
    src/geometry/MeshGeometry.cpp
    src/geometry/VolumetricGeometry.cpp
    src/MaterialManager.cpp
    # --- Undo/Redo System ---
    src/commands/CommandManager.cpp
    src/commands/PushPullCommand.cpp
    src/commands/MoveCommand.cpp
    src/commands/CreateLineCommand.cpp
    src/commands/MoveSubObjectCommand.cpp
    src/commands/SculptCommand.cpp
    src/commands/VoxelizeCommand.cpp
    # Geometry implementations
)

# Указываем, где находятся публичные заголовочные файлы
target_include_directories(engine PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# --- Находим и подключаем зависимости ---
# Находим пакеты, от которых зависит engine
find_package(fmt CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(OpenMP REQUIRED)

# --- NEW: Find libigl and its dependencies (Eigen) ---
find_package(libigl CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
message(STATUS "Found libigl and Eigen3 via vcpkg.")

# --- NEW: Find OpenVDB and TBB for sparse voxel storage ---
find_package(OpenVDB CONFIG REQUIRED)
find_package(TBB CONFIG REQUIRED)
message(STATUS "Found OpenVDB and TBB via vcpkg.")

# Подключаем библиотеки. PUBLIC означает, что любой, кто использует "engine",
# также автоматически получит пути к fmt и glm. Это решит проблему с линковкой fmt.
target_link_libraries(engine PUBLIC
    cad_kernel # Наша обертка над OCCT
    fmt::fmt
    glm::glm
    # --- NEW: Link against libigl and Eigen3 ---
    igl::igl_core
    ${LIBIGL_COPYLEFT_LIBRARIES}
    Eigen3::Eigen
    OpenMP::OpenMP_CXX
    # --- NEW: Link against OpenVDB and TBB ---
    OpenVDB::openvdb
    TBB::tbb
)

# --- GPU Acceleration (if enabled) ---
if(TARGET urbaxio_gpu)
    target_link_libraries(engine PUBLIC urbaxio_gpu)
    message(STATUS "Engine: GPU acceleration linked")
endif()