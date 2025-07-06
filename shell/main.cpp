// --- Includes ---
#include <engine/engine.h>
#include <engine/scene.h>
#include <engine/scene_object.h>
#include <cad_kernel/cad_kernel.h>

#include "camera.h"
#include "input_handler.h"
#include "renderer.h"

#include <SDL2/SDL.h>
#include <glad/glad.h>

#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_opengl3.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <set>
#include <map>
#include <list>
#include <cstdint>
#include <cstddef>
#include <charconv>
#include <limits> // Required for numeric_limits 2222


// --- GPU Mesh Upload Helper ---
bool UploadMeshToGPU(Urbaxio::Engine::SceneObject& object) { /* ... */ const Urbaxio::CadKernel::MeshBuffers& mesh = object.get_mesh_buffers(); if (mesh.isEmpty() || mesh.normals.empty()) { if (mesh.normals.empty() && !mesh.vertices.empty()) { std::cerr << "UploadMeshToGPU: Mesh for object " << object.get_id() << " is missing normals!" << std::endl; } return false; } if (object.vao != 0) glDeleteVertexArrays(1, &object.vao); if (object.vbo_vertices != 0) glDeleteBuffers(1, &object.vbo_vertices); if (object.vbo_normals != 0) glDeleteBuffers(1, &object.vbo_normals); if (object.ebo != 0) glDeleteBuffers(1, &object.ebo); object.vao = object.vbo_vertices = object.vbo_normals = object.ebo = 0; object.index_count = 0; glGenVertexArrays(1, &object.vao); if (object.vao == 0) return false; glBindVertexArray(object.vao); glGenBuffers(1, &object.vbo_vertices); if (object.vbo_vertices == 0) { glDeleteVertexArrays(1, &object.vao); object.vao = 0; return false; } glBindBuffer(GL_ARRAY_BUFFER, object.vbo_vertices); glBufferData(GL_ARRAY_BUFFER, mesh.vertices.size() * sizeof(float), mesh.vertices.data(), GL_STATIC_DRAW); glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0); glEnableVertexAttribArray(0); glGenBuffers(1, &object.vbo_normals); if (object.vbo_normals == 0) { glDeleteBuffers(1, &object.vbo_vertices); glDeleteVertexArrays(1, &object.vao); object.vao = object.vbo_vertices = 0; return false; } glBindBuffer(GL_ARRAY_BUFFER, object.vbo_normals); glBufferData(GL_ARRAY_BUFFER, mesh.normals.size() * sizeof(float), mesh.normals.data(), GL_STATIC_DRAW); glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0); glEnableVertexAttribArray(1); glGenBuffers(1, &object.ebo); if (object.ebo == 0) { glDeleteBuffers(1, &object.vbo_normals); glDeleteBuffers(1, &object.vbo_vertices); glDeleteVertexArrays(1, &object.vao); object.vao = object.vbo_vertices = object.vbo_normals = 0; return false; } glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, object.ebo); glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh.indices.size() * sizeof(unsigned int), mesh.indices.data(), GL_STATIC_DRAW); object.index_count = static_cast<GLsizei>(mesh.indices.size()); glBindVertexArray(0); glBindBuffer(GL_ARRAY_BUFFER, 0); glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0); std::cout << "UploadMeshToGPU: Successfully uploaded mesh for object " << object.get_id() << std::endl; return true; }

// Tolerances and Comparator (using the one from Urbaxio namespace now)
const float POINT_EQUALITY_TOLERANCE_MAIN = Urbaxio::SCENE_POINT_EQUALITY_TOLERANCE;
const float POINT_EQUALITY_TOLERANCE_SQ_MAIN = POINT_EQUALITY_TOLERANCE_MAIN * POINT_EQUALITY_TOLERANCE_MAIN;
const float COPLANARITY_TOLERANCE_MAIN = 1e-4f;

bool ArePointsEqualMain(const glm::vec3& p1, const glm::vec3& p2) {
    return glm::distance2(p1, p2) < POINT_EQUALITY_TOLERANCE_SQ_MAIN;
}


int main(int argc, char* argv[]) {
    std::cout << "Shell: Starting Urbaxio Application..." << std::endl;
    // --- Initialization ---
    initialize_engine(); Urbaxio::Engine::Scene* scene_ptr = reinterpret_cast<Urbaxio::Engine::Scene*>(get_engine_scene()); if (!scene_ptr) return 1; if (SDL_Init(SDL_INIT_VIDEO) != 0) return 1; const char* glsl_version = "#version 330 core"; SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_DEBUG_FLAG); SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3); SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3); SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE); SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1); SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24); SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8); SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI | SDL_WINDOW_SHOWN); SDL_Window* window = SDL_CreateWindow("Urbaxio", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720, window_flags); if (!window) { SDL_Quit(); return 1; } SDL_GLContext gl_context = SDL_GL_CreateContext(window); if (!gl_context) { SDL_DestroyWindow(window); SDL_Quit(); return 1; } SDL_GL_MakeCurrent(window, gl_context); SDL_GL_SetSwapInterval(1); if (!gladLoadGLLoader((GLADloadproc)SDL_GL_GetProcAddress)) { return 1; } std::cout << "Shell: OpenGL Initialized: V:" << glGetString(GL_VERSION) << std::endl; IMGUI_CHECKVERSION(); ImGui::CreateContext(); ImGuiIO& io = ImGui::GetIO(); io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; ImGui::StyleColorsDark(); if (!ImGui_ImplSDL2_InitForOpenGL(window, gl_context)) return 1; if (!ImGui_ImplOpenGL3_Init(glsl_version)) return 1; std::cout << "Shell: All subsystems initialized." << std::endl;

    Urbaxio::Renderer renderer; if (!renderer.Initialize()) { return 1; }
    Urbaxio::Camera camera; Urbaxio::InputHandler inputHandler;
    ImVec4 clear_color = ImVec4(0.18f, 0.18f, 0.22f, 1.00f); int object_counter = 0; glm::vec3 objectColor(0.6f, 0.7f, 0.9f); glm::vec3 lightDirection = glm::normalize(glm::vec3(0.8f, 1.0f, -0.6f)); glm::vec3 lightColor = glm::vec3(1.0f, 1.0f, 1.0f); float ambientStrength = 0.15f; float gridLineWidth = 2.0f; float axisLineWidth = 4.0f; glm::vec4 splatColor = glm::vec4(1.0f, 0.5f, 0.2f, 0.8f); float splatBlurStrength = 10.0f; bool showGrid = true; bool showAxes = true; float maxLineWidth = renderer.GetMaxLineWidth();
    // --- Selection State ---
    uint64_t selectedObjId = 0;
    std::vector<size_t> selectedTriangleIndices;
    std::vector<size_t> selectedLineIndices;
    glm::vec3 selectionHighlightColor = glm::vec3(0.6f, 0.8f, 1.0f);

    // --- Tool State ---
    bool isDrawingLineMode = false;
    bool isPushPullMode = false;
    bool isPushPullActive = false; // Is a Push/Pull operation currently in progress?

    // --- Push/Pull State ---
    uint64_t hoveredObjId = 0;
    std::vector<size_t> hoveredFaceTriangleIndices;
    glm::vec3 hoverHighlightColor = glm::vec3(0.4f, 0.9f, 1.0f); // Light cyan
    float pushPullCurrentDistance = 0.0f; // This will be updated by InputHandler

    // --- Drawing State ---
    bool isPlacingFirstPoint = false;
    bool isPlacingSecondPoint = false;
    glm::vec3 currentLineStartPoint(0.0f);
    glm::vec3 currentRubberBandEnd(0.0f);
    Urbaxio::SnapResult currentSnap;
    char lineLengthInputBuf[64] = "";
    float lineLengthValue = 0.0f;

    bool should_quit = false; std::cout << "Shell: >>> Entering main loop..." << std::endl;
    while (!should_quit) {
        int display_w, display_h; SDL_GetWindowSize(window, &display_w, &display_h);
        if (isDrawingLineMode && !isPlacingFirstPoint && !isPlacingSecondPoint) { isPlacingFirstPoint = true; } else if (!isDrawingLineMode && (isPlacingFirstPoint || isPlacingSecondPoint)) { isPlacingFirstPoint = false; isPlacingSecondPoint = false; lineLengthInputBuf[0] = '\0'; lineLengthValue = 0.0f; }
        
        inputHandler.ProcessEvents(camera, should_quit, window, display_w, display_h, selectedObjId, selectedTriangleIndices, selectedLineIndices, isDrawingLineMode, isPushPullMode, isPushPullActive, hoveredObjId, hoveredFaceTriangleIndices, pushPullCurrentDistance, isPlacingFirstPoint, isPlacingSecondPoint, currentLineStartPoint, scene_ptr, currentRubberBandEnd, currentSnap, lineLengthInputBuf, lineLengthValue);
        
        if (isPushPullActive) {
            Urbaxio::Engine::SceneObject* obj = scene_ptr->get_object_by_id(inputHandler.GetPushPullObjectId());
            if (obj) {
                renderer.UpdatePushPullPreview(*obj, inputHandler.GetPushPullFaceIndices(), inputHandler.GetPushPullNormal(), pushPullCurrentDistance);
            }
        } else {
            // Clear preview when not active
            renderer.UpdatePushPullPreview(Urbaxio::Engine::SceneObject(0, ""), {}, {}, 0.0f);
        }

        static bool wasPlacingSecondPoint = false; if (wasPlacingSecondPoint && !isPlacingSecondPoint) { /* ... */ } wasPlacingSecondPoint = isPlacingSecondPoint;
        if (should_quit) break;
        if (scene_ptr) { renderer.UpdateUserLinesBuffer(scene_ptr->GetLineSegments(), selectedLineIndices); }

        // GPU Upload for newly created face objects
        if (scene_ptr) {
            std::vector<Urbaxio::Engine::SceneObject*> all_objects = scene_ptr->get_all_objects();
            for (Urbaxio::Engine::SceneObject* obj : all_objects) {
                if (obj && obj->has_mesh() && obj->vao == 0) { // Has mesh, but not yet on GPU
                    if (!UploadMeshToGPU(*obj)) {
                        std::cerr << "Shell: Main loop failed to upload mesh for object " << obj->get_id() << std::endl;
                    }
                }
            }
        }

        ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplSDL2_NewFrame(); ImGui::NewFrame();

        { // Urbaxio Controls Window
            ImGui::Begin("Urbaxio Controls");
            // ... (FPS, Create Box, Colors, Lighting, View Options, Splat Test, Drawing - same as before) ...
            ImGui::Text("App avg %.3f ms/f (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate); ImGui::Separator(); if (ImGui::Button("Create Box Object")) { object_counter++; std::string box_name = "Box_" + std::to_string(object_counter); Urbaxio::Engine::SceneObject* new_box = scene_ptr->create_box_object(box_name, 10.0, 20.0, 5.0); if (new_box && new_box->has_mesh()) { /* GPU upload handled by main loop now */ } else { if (!new_box) { std::cerr << "Shell: Failed to create SceneObject for '" << box_name << "'." << std::endl; } else { std::cerr << "Shell: Failed to triangulate or mesh is empty for '" << box_name << "'." << std::endl; } } } ImGui::Separator(); ImGui::ColorEdit3("Object Color", (float*)&objectColor); ImGui::ColorEdit3("Background Color", (float*)&clear_color); ImGui::Separator(); ImGui::Text("Lighting:"); ImGui::SliderFloat("Ambient Strength", &ambientStrength, 0.0f, 1.0f); static glm::vec3 lightDirInput = lightDirection; if (ImGui::SliderFloat3("Light Direction", glm::value_ptr(lightDirInput), -1.0f, 1.0f)) { if (glm::length(lightDirInput) > 1e-6f) { lightDirection = glm::normalize(lightDirInput); } } ImGui::ColorEdit3("Light Color", glm::value_ptr(lightColor)); ImGui::Separator(); ImGui::Text("View Options:"); ImGui::Checkbox("Show Grid", &showGrid); ImGui::SameLine(); ImGui::Checkbox("Show Axes", &showAxes); ImGui::SliderFloat("Grid Line Width", &gridLineWidth, 1.0f, maxLineWidth); ImGui::SliderFloat("Axis Line Width", &axisLineWidth, 1.0f, maxLineWidth); ImGui::Separator(); ImGui::Text("Gaussian Splat Test:"); ImGui::ColorEdit4("Splat Color", glm::value_ptr(splatColor), ImGuiColorEditFlags_AlphaBar); ImGui::SliderFloat("Splat Blur Strength", &splatBlurStrength, 1.0f, 50.0f); ImGui::Separator();
            ImGui::Text("Tools:");
            if (ImGui::Checkbox("Draw Line Mode", &isDrawingLineMode)) {
                if (isDrawingLineMode) { isPushPullMode = false; isPushPullActive = false; }
            }
            ImGui::SameLine();
            if (ImGui::Checkbox("Push/Pull Mode", &isPushPullMode)) {
                if (isPushPullMode) { isDrawingLineMode = false; } else { isPushPullActive = false; }
            }
            if (ImGui::Button("Clear Lines") && scene_ptr) { scene_ptr->ClearUserLines(); renderer.UpdateUserLinesBuffer(scene_ptr->GetLineSegments(), selectedLineIndices); isPlacingFirstPoint = isDrawingLineMode; isPlacingSecondPoint = false; lineLengthInputBuf[0] = '\0'; selectedLineIndices.clear(); }

            if (!selectedLineIndices.empty()) {
                if (ImGui::Button("Show Selected Lines (Debug)")) {
                    std::cout << "DEBUG: Currently selected line segment indices: ";
                    for (size_t index : selectedLineIndices) { std::cout << index << " "; }
                    std::cout << std::endl;
                }
            }

            if (isPlacingSecondPoint) { ImGui::Separator(); ImGui::Text("Length: %s", lineLengthInputBuf); ImGui::Separator(); }
            ImGui::Text("Scene Info:");
            ImGui::Text("Selected Object ID: %llu", selectedObjId);
            if (selectedObjId != 0) {
                 ImGui::Text("Selected Triangles: %zu", selectedTriangleIndices.size());
            }

            const char* snapTypeName = "None"; if (currentSnap.snapped) { switch (currentSnap.type) { case Urbaxio::SnapType::ENDPOINT: snapTypeName = "Endpoint"; break; case Urbaxio::SnapType::ORIGIN:   snapTypeName = "Origin"; break; case Urbaxio::SnapType::AXIS_X:   snapTypeName = "On Axis X"; break; case Urbaxio::SnapType::AXIS_Y:   snapTypeName = "On Axis Y"; break; case Urbaxio::SnapType::AXIS_Z:   snapTypeName = "On Axis Z"; break; default: snapTypeName = "Unknown"; break; } } ImGui::Text("Current Snap: %s (%.2f, %.2f, %.2f)", snapTypeName, currentSnap.worldPoint.x, currentSnap.worldPoint.y, currentSnap.worldPoint.z);
            ImGui::Text("Scene Objects:"); if (scene_ptr) { std::vector<Urbaxio::Engine::SceneObject*> objects = scene_ptr->get_all_objects(); if (objects.empty()) { ImGui::TextDisabled("(No objects yet)"); } else { ImGui::BeginChild("ObjectList", ImVec2(0, 100), true, ImGuiWindowFlags_HorizontalScrollbar); for (const auto* obj : objects) { if (obj) { ImGui::BulletText("%s (ID:%llu)%s%s%s", obj->get_name().c_str(), obj->get_id(), obj->has_shape() ? " [Geo]" : "", obj->has_mesh() ? " [Mesh]" : "", (obj->vao != 0) ? " [GPU]" : ""); } } ImGui::EndChild(); } } else { ImGui::TextDisabled("(Scene pointer is null)"); }
            ImGui::End();
        }

        ImGui::Render();
        renderer.SetViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        renderer.RenderFrame(window, camera, scene_ptr, objectColor, lightDirection, lightColor, ambientStrength, showGrid, showAxes, gridLineWidth, axisLineWidth, splatColor, splatBlurStrength, selectedObjId, selectedTriangleIndices, selectedLineIndices, selectionHighlightColor, hoveredObjId, hoveredFaceTriangleIndices, hoverHighlightColor, isPlacingSecondPoint, currentLineStartPoint, currentRubberBandEnd, currentSnap, ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);
    }

    std::cout << "Shell: <<< Exiting main loop." << std::endl;
    std::cout << "Shell: Cleaning up..." << std::endl; /* ... Cleanup ... */ if (scene_ptr) { std::vector<Urbaxio::Engine::SceneObject*> objects_to_clean = scene_ptr->get_all_objects(); for (auto* obj : objects_to_clean) { if (obj && obj->vao != 0) glDeleteVertexArrays(1, &obj->vao); if (obj && obj->vbo_vertices != 0) glDeleteBuffers(1, &obj->vbo_vertices); if (obj && obj->vbo_normals != 0) glDeleteBuffers(1, &obj->vbo_normals); if (obj && obj->ebo != 0) glDeleteBuffers(1, &obj->ebo); } } ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplSDL2_Shutdown(); ImGui::DestroyContext(); SDL_GL_DeleteContext(gl_context); SDL_DestroyWindow(window); SDL_Quit(); std::cout << "Shell: Urbaxio Application finished gracefully." << std::endl; return 0;
}