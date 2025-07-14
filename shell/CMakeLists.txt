# Set the project name
project(UrbaxioShell)

# Enable RC language for Windows resources
if(WIN32)
    enable_language(RC)
    set(RESOURCE_FILES "${CMAKE_SOURCE_DIR}/resources/urbaxio.rc")
else()
    set(RESOURCE_FILES "")
endif()

# Define the executable target FIRST
add_executable(Urbaxio
    main.cpp
    src/camera.cpp
    src/input_handler.cpp
    src/renderer.cpp
    src/snapping.cpp
    # Add ImGui backend implementation files directly to the compilation
    "C:/vcpkg/buildtrees/imgui/src/v1.91.9-afb09617a6.clean/backends/imgui_impl_sdl2.cpp"
    "C:/vcpkg/buildtrees/imgui/src/v1.91.9-afb09617a6.clean/backends/imgui_impl_opengl3.cpp"
    # Add Windows resources
    ${RESOURCE_FILES}
)

# NOW specify include directories for the defined target "Urbaxio"
target_include_directories(Urbaxio PRIVATE
    include # Add the shell/include directory
    # Also include the backends path for its internal includes
    "C:/vcpkg/buildtrees/imgui/src/v1.91.9-afb09617a6.clean/backends"
)

# --- Find dependencies needed directly by shell ---
find_package(SDL2 CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(OpenGL REQUIRED)
find_package(imgui CONFIG REQUIRED)

# --- Link the application with libraries ---
# OCCT and glm are inherited from engine/cad_kernel now
target_link_libraries(Urbaxio PRIVATE
    engine          # Our engine library
    cad_kernel      # Our CAD kernel wrapper. It brings OCCT libs with it.
    tools           # Our new tools library
    SDL2::SDL2main
    SDL2::SDL2
    glad::glad
    OpenGL::GL
    imgui::imgui
)

# --- Copy OpenCascade DLLs after build ---
# This remains important for running the executable
set(VCPKG_GLOBAL_BIN_DIR "${CMAKE_BINARY_DIR}/vcpkg_installed/${VCPKG_TARGET_TRIPLET}/bin")
set(VCPKG_GLOBAL_DEBUG_BIN_DIR "${CMAKE_BINARY_DIR}/vcpkg_installed/${VCPKG_TARGET_TRIPLET}/debug/bin")
add_custom_command(TARGET Urbaxio POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "Copying OCCT DLLs for $<CONFIG>..."
    COMMAND $<$<CONFIG:Debug>:${CMAKE_COMMAND}> -E copy_directory_if_different "${VCPKG_GLOBAL_DEBUG_BIN_DIR}/" "$<TARGET_FILE_DIR:Urbaxio>/"
    COMMAND $<$<CONFIG:Release>:${CMAKE_COMMAND}> -E copy_directory_if_different "${VCPKG_GLOBAL_BIN_DIR}/" "$<TARGET_FILE_DIR:Urbaxio>/"
    COMMENT "Copying OpenCascade DLLs to executable directory..."
    VERBATIM
)
message(STATUS "Added post-build command to copy OCCT DLLs.")

# --- Resources configuration ---
if(WIN32 AND RESOURCE_FILES)
    message(STATUS "Windows resources enabled. Logo will be embedded in executable.")
    message(STATUS "Resource file: ${RESOURCE_FILES}")
    # Set working directory for resource compiler to find the ICO file
    set_property(SOURCE ${RESOURCE_FILES} PROPERTY COMPILE_FLAGS "/I\"${CMAKE_SOURCE_DIR}/resources\"")
else()
    message(STATUS "No Windows resources configured.")
endif()

message(STATUS "Configuring Urbaxio executable...")
# --- Check found libraries (Corrected multi-line format) ---
if(TARGET SDL2::SDL2)
    message(STATUS "Found SDL2 library via vcpkg.")
endif()
if(TARGET SDL2::SDL2main)
    message(STATUS "Found SDL2main library via vcpkg.")
endif()
if(TARGET glad::glad)
    message(STATUS "Found glad library via vcpkg.")
endif()
if(TARGET OpenGL::GL)
    message(STATUS "Found OpenGL library.")
endif()
if(TARGET imgui::imgui)
    message(STATUS "Found Dear ImGui library via vcpkg.")
endif()