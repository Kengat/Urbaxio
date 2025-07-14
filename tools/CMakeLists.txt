# Set the project name for the tools library
project(UrbaxioTools)

# Create a static library named 'tools'
add_library(tools STATIC
    src/SelectTool.cpp
    src/LineTool.cpp
    src/PushPullTool.cpp
    src/ToolManager.cpp
)

# Expose the 'include' directory for other modules
target_include_directories(tools PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../shell/include"
)

# Tools depend on the engine and cad_kernel
target_link_libraries(tools PRIVATE
    engine
    cad_kernel
)

message(STATUS "Configuring Tools library...") 