# Определяем статическую библиотеку "engine"
add_library(engine STATIC
    src/engine_main.cpp
    src/scene.cpp
    src/scene_object.cpp
    # --- Undo/Redo System ---
    src/commands/CommandManager.cpp
    src/commands/PushPullCommand.cpp
    src/commands/MoveCommand.cpp
    src/commands/CreateLineCommand.cpp
    src/commands/MoveSubObjectCommand.cpp # <-- ДОБАВИТЬ ЭТУ СТРОКУ
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

# Подключаем библиотеки. PUBLIC означает, что любой, кто использует "engine",
# также автоматически получит пути к fmt и glm. Это решит проблему с линковкой fmt.
target_link_libraries(engine PUBLIC
    cad_kernel # Наша обертка над OCCT
    fmt::fmt
    glm::glm
)