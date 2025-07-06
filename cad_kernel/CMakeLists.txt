# Определяем статическую библиотеку "cad_kernel"
add_library(cad_kernel STATIC
    src/cad_kernel.cpp
    src/TriangulateShape.cpp
)

# Указываем, где находятся публичные заголовочные файлы
target_include_directories(cad_kernel PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Подключаем к нашей библиотеке найденные вручную библиотеки OpenCASCADE
# PUBLIC здесь важно, чтобы shell, который использует cad_kernel, тоже смог их "увидеть"
target_link_libraries(cad_kernel PUBLIC ${OpenCASCADE_LIBRARIES})