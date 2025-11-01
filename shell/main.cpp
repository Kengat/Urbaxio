// Force dGPU usage on hybrid laptops (NVIDIA/AMD)
extern "C" {
    __declspec(dllexport) unsigned long NvOptimusEnablement = 0x00000001;        // NVIDIA Optimus
    __declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;          // AMD PowerXpress
}

// Enable GLM experimental features (must be before any GLM includes via headers)
#define GLM_ENABLE_EXPERIMENTAL

// --- Includes ---
#include <engine/engine.h>
#include <engine/scene.h>
#include <engine/scene_object.h>
#include <cad_kernel/cad_kernel.h>
#include <tools/ToolManager.h>
#include <tools/SelectTool.h>
#include <tools/LineTool.h>
#include <tools/MoveTool.h>
#include <tools/PushPullTool.h>

#include "camera.h"
#include "input_handler.h"
#include "renderer.h"
#include "VRManager.h"

#include <SDL2/SDL.h>
#include <glad/glad.h>

#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_opengl3.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/matrix_decompose.hpp>

#include <fmt/core.h>

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <set>
#include <map>
#include <cstdint>

#include "TextRenderer.h"
#include <filesystem>
// --- VR menu interaction helpers ---
#include <glm/gtx/intersect.hpp>
#include <limits>

// --- OCCT includes for capsule generation (moved from engine) ---
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <TopoDS_Shape.hxx>
#include <BRepBuilderAPI_Transform.hxx>

namespace { // Anonymous namespace for helpers

    // Helper to create a model matrix from an OpenXR pose
    glm::mat4 XrPoseToModelMatrix(const XrPosef& pose) {
        glm::quat orientation(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        glm::vec3 position(pose.position.x, pose.position.y, pose.position.z);
        glm::mat4 rotationMatrix = glm::toMat4(orientation);
        glm::mat4 translationMatrix = glm::translate(glm::mat4(1.0f), position);
        return translationMatrix * rotationMatrix;
    }

    // Transform a 3D point by a 4x4 matrix
    glm::vec3 TransformPoint(const glm::mat4& M, const glm::vec3& p) {
        return glm::vec3(M * glm::vec4(p, 1.0f));
    }

    // Helper to interpolate color from green to cyan
    glm::vec3 MixColorFromPress(float t) {
        return glm::mix(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 1.0f, 1.0f), std::clamp(t, 0.0f, 1.0f));
    }

    // --- Capsule generation logic is now in the shell ---
    Urbaxio::CadKernel::MeshBuffers CreateCapsuleMesh(float radius, float height) {
        try {
            // --- FIX: Offset the cylinder's base down by half its height to center it ---
            gp_Pnt center(0.0, 0.0, -height / 2.0);
            gp_Dir z_dir(0, 0, 1);
            gp_Ax2 axis(center, z_dir);
            
            TopoDS_Shape cylinder = BRepPrimAPI_MakeCylinder(axis, radius, height);

            // Create a single sphere at the origin, to be moved
            TopoDS_Shape sphere = BRepPrimAPI_MakeSphere(gp_Pnt(0,0,0), radius);

            // Transformations to move the sphere to the ends of the cylinder body
            gp_Trsf top_trsf, bot_trsf;
            top_trsf.SetTranslation(gp_Vec(0, 0, height / 2.0));
            bot_trsf.SetTranslation(gp_Vec(0, 0, -height / 2.0));

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

// --- GPU Mesh Upload Helper ---
bool UploadMeshToGPU(Urbaxio::Engine::SceneObject& object) { /* ... */ const Urbaxio::CadKernel::MeshBuffers& mesh = object.get_mesh_buffers(); if (mesh.isEmpty() || mesh.normals.empty()) { if (mesh.normals.empty() && !mesh.vertices.empty()) { std::cerr << "UploadMeshToGPU: Mesh for object " << object.get_id() << " is missing normals!" << std::endl; } return false; } if (object.vao != 0) glDeleteVertexArrays(1, &object.vao); if (object.vbo_vertices != 0) glDeleteBuffers(1, &object.vbo_vertices); if (object.vbo_normals != 0) glDeleteBuffers(1, &object.vbo_normals); if (object.ebo != 0) glDeleteBuffers(1, &object.ebo); object.vao = object.vbo_vertices = object.vbo_normals = object.ebo = 0; object.index_count = 0; glGenVertexArrays(1, &object.vao); if (object.vao == 0) return false; glBindVertexArray(object.vao); glGenBuffers(1, &object.vbo_vertices); if (object.vbo_vertices == 0) { glDeleteVertexArrays(1, &object.vao); object.vao = 0; return false; } glBindBuffer(GL_ARRAY_BUFFER, object.vbo_vertices); glBufferData(GL_ARRAY_BUFFER, mesh.vertices.size() * sizeof(float), mesh.vertices.data(), GL_STATIC_DRAW); glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0); glEnableVertexAttribArray(0); glGenBuffers(1, &object.vbo_normals); if (object.vbo_normals == 0) { glDeleteBuffers(1, &object.vbo_vertices); glDeleteVertexArrays(1, &object.vao); object.vao = object.vbo_vertices = 0; return false; } glBindBuffer(GL_ARRAY_BUFFER, object.vbo_normals); glBufferData(GL_ARRAY_BUFFER, mesh.normals.size() * sizeof(float), mesh.normals.data(), GL_STATIC_DRAW); glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0); glEnableVertexAttribArray(1); glGenBuffers(1, &object.ebo); if (object.ebo == 0) { glDeleteBuffers(1, &object.vbo_normals); glDeleteBuffers(1, &object.vbo_vertices); glDeleteVertexArrays(1, &object.vao); object.vao = object.vbo_vertices = object.vbo_normals = 0; return false; } glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, object.ebo); glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh.indices.size() * sizeof(unsigned int), mesh.indices.data(), GL_STATIC_DRAW); object.index_count = static_cast<GLsizei>(mesh.indices.size()); glBindVertexArray(0); glBindBuffer(GL_ARRAY_BUFFER, 0); glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0); std::cout << "UploadMeshToGPU: Successfully uploaded mesh for object " << object.get_id() << std::endl; return true; }

int main(int argc, char* argv[]) {
    std::cout << "Shell: Starting Urbaxio Application..." << std::endl;
    // --- Initialization ---
    initialize_engine(); Urbaxio::Engine::Scene* scene_ptr = reinterpret_cast<Urbaxio::Engine::Scene*>(get_engine_scene()); if (!scene_ptr) return 1; if (SDL_Init(SDL_INIT_VIDEO) != 0) return 1; const char* glsl_version = "#version 430 core"; SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_DEBUG_FLAG); SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4); SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3); SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE); SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1); SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24); SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8); SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI | SDL_WINDOW_SHOWN); SDL_Window* window = SDL_CreateWindow("Urbaxio", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720, window_flags); if (!window) { SDL_Quit(); return 1; } SDL_GLContext gl_context = SDL_GL_CreateContext(window); if (!gl_context) { SDL_DestroyWindow(window); SDL_Quit(); return 1; } SDL_GL_MakeCurrent(window, gl_context); SDL_GL_SetSwapInterval(1); if (!gladLoadGLLoader((GLADloadproc)SDL_GL_GetProcAddress)) { return 1; } std::cout << "Shell: OpenGL Initialized: V:" << glGetString(GL_VERSION) << std::endl; std::cout << "Shell: OpenGL Renderer: " << glGetString(GL_RENDERER) << std::endl; std::cout << "Shell: OpenGL Vendor: " << glGetString(GL_VENDOR) << std::endl; IMGUI_CHECKVERSION(); ImGui::CreateContext(); ImGuiIO& io = ImGui::GetIO(); io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; ImGui::StyleColorsDark(); if (!ImGui_ImplSDL2_InitForOpenGL(window, gl_context)) return 1; if (!ImGui_ImplOpenGL3_Init(glsl_version)) return 1;     std::cout << "Shell: All subsystems initialized." << std::endl;

    // --- VR Initialization ---
    bool vr_mode = true;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--novr") == 0) {
            vr_mode = false;
            break;
        }
    }
    std::unique_ptr<Urbaxio::VRManager> vrManager = nullptr;
    if (vr_mode) {
        vrManager = std::make_unique<Urbaxio::VRManager>();
        if (!vrManager->Initialize(window)) {
            std::cerr << "Failed to initialize VR. Falling back to 2D mode." << std::endl;
            vr_mode = false;
            vrManager.reset();
        }
    }

    Urbaxio::Renderer renderer; if (!renderer.Initialize()) { return 1; }
    Urbaxio::TextRenderer textRenderer;
    {
        std::string fontJsonPath = "../../resources/futura_demi.json";
        std::string fontPngPath = "../../resources/futura_demi.png";
        if (!std::filesystem::exists(fontJsonPath)) {
            fontJsonPath = "../../../resources/futura_demi.json";
            fontPngPath = "../../../resources/futura_demi.png";
        }
        if (!textRenderer.Initialize(fontJsonPath, fontPngPath)) { /* silent fail */ }
    }
    Urbaxio::Camera camera; Urbaxio::InputHandler inputHandler;
    Urbaxio::SnappingSystem snappingSystem;
    int object_counter = 0;
    
    // --- Appearance Settings ---
    ImVec4 clear_color = ImVec4(0.13f, 0.13f, 0.18f, 1.00f);
    glm::vec3 objectColor(0.8f, 0.85f, 0.9f);
    glm::vec3 lightColor = glm::vec3(0.618f, 0.858f, 0.844f);
    float ambientStrength = 0.267f;
    bool showGrid = true; bool showAxes = true;
    glm::vec3 gridColor(58.f / 255.f, 82.f / 255.f, 105.f / 255.f);
    glm::vec4 axisColorX(223.f / 255.f, 62.f / 255.f, 86.f / 255.f, 1.0f);
    glm::vec4 axisColorY(58.f / 255.f, 223.f / 255.f, 150.f / 255.f, 1.0f);
    glm::vec4 axisColorZ(63.f / 255.f, 56.f / 255.f, 184.f / 255.f, 1.0f);
    float axisLineWidth = 2.302f;
    glm::vec4 positiveAxisFadeColor(72.f / 255.f, 94.f / 255.f, 96.f / 255.f, 1.0f);
    float negAxisLineWidth = 1.698f;
    glm::vec4 negativeAxisFadeColor(50.f / 255.f, 81.f / 255.f, 86.f / 255.f, 102.f / 255.f);
    float cursorRadius = 15.0f;
    float effectIntensity = 0.8f;
    
    // --- Other Settings ---
    float maxLineWidth = renderer.GetMaxLineWidth();
    bool show_style_editor = false;

    // --- Core State Variables ---
    uint64_t selectedObjId = 0;
    std::vector<size_t> selectedTriangleIndices;
    std::set<uint64_t> selectedLineIDs;
    glm::vec3 selectionHighlightColor = glm::vec3(1.0f, 225.0f / 255.0f, 84.0f / 255.0f);

    uint64_t hoveredObjId = 0;
    std::vector<size_t> hoveredFaceTriangleIndices;
    glm::vec3 hoverHighlightColor = glm::vec3(0.4f, 0.9f, 1.0f); // Light cyan
    
    // --- Tool Manager Setup ---
    int display_w, display_h;
    SDL_GetWindowSize(window, &display_w, &display_h);

    bool shiftDown = false;
    bool ctrlDown = false;

    Urbaxio::Tools::ToolContext toolContext;
    toolContext.scene = scene_ptr;
    toolContext.camera = &camera;
    toolContext.window = window;
    toolContext.display_w = &display_w;
    toolContext.display_h = &display_h;
    toolContext.shiftDown = &shiftDown;
    toolContext.ctrlDown = &ctrlDown;
    toolContext.selectedObjId = &selectedObjId;
    toolContext.selectedTriangleIndices = &selectedTriangleIndices;
    toolContext.selectedLineIDs = &selectedLineIDs;
    toolContext.hoveredObjId = &hoveredObjId;
    toolContext.hoveredFaceTriangleIndices = &hoveredFaceTriangleIndices;
    
    Urbaxio::Tools::ToolManager toolManager(toolContext);

    // --- Marker settings ---
    static float capsuleRadius = 0.5f;
    static float capsuleHeight10m = 3.2f;
    static float capsuleHeight5m = 1.4f;

    // --- Create shell-specific markers ---
    auto* capsule_marker_10m = scene_ptr->create_object("UnitCapsuleMarker10m");
    if (capsule_marker_10m) {
        Urbaxio::CadKernel::MeshBuffers mesh = CreateCapsuleMesh(capsuleRadius, capsuleHeight10m);
        capsule_marker_10m->set_mesh_buffers(std::move(mesh));
    }
    
    auto* capsule_marker_5m = scene_ptr->create_object("UnitCapsuleMarker5m");
    if (capsule_marker_5m) {
        Urbaxio::CadKernel::MeshBuffers mesh = CreateCapsuleMesh(capsuleRadius, capsuleHeight5m);
        capsule_marker_5m->set_mesh_buffers(std::move(mesh));
    }

    // --- NEW: Create VR Controller Visuals ---
    Urbaxio::Engine::SceneObject* leftControllerVisual = nullptr;
    Urbaxio::Engine::SceneObject* rightControllerVisual = nullptr;
    if (vr_mode) {
        // Create 1x1x1 unit cubes. The scaling will be done in the render loop.
        leftControllerVisual = scene_ptr->create_box_object("LeftControllerVisual", 1.0, 1.0, 1.0);
        rightControllerVisual = scene_ptr->create_box_object("RightControllerVisual", 1.0, 1.0, 1.0);
    }
    // --- VR menu interaction state ---
    int hoveredToolIndex = -1;
    std::vector<float> toolMenuAlphas;
    Urbaxio::Engine::SceneObject* menuSphere = nullptr; // For rendering widgets

    int hoveredNumpadKey = -1; // -1 for none, 0-9 for digits, 10 for '.', 11 for 'Confirm'
    std::string numpadInput = "0";

    glm::mat4 numpadTransform = glm::mat4(1.0f); // Transform for the entire numpad panel

    glm::mat4 numpadOffsetTransform = glm::mat4(1.0f); // Relative offset from the controller
    bool numpadInitialized = false;

    bool isGrabbingNumpad = false;
    glm::mat4 grabbedNumpadInitialTransform;
    glm::mat4 grabbedControllerInitialTransform;
    bool isHoveringGrabHandle = false;

    bool should_quit = false; std::cout << "Shell: >>> Entering main loop..." << std::endl;
    while (!should_quit) {
        SDL_GetWindowSize(window, &display_w, &display_h);
        
        const Uint8* keyboardState = SDL_GetKeyboardState(NULL);
        // For 2D mode, read keyboard state; for VR mode, state will be updated in VR section after PollActions
        if (!vr_mode || !vrManager || !vrManager->IsInitialized()) {
            shiftDown = keyboardState[SDL_SCANCODE_LSHIFT] || keyboardState[SDL_SCANCODE_RSHIFT];
            ctrlDown = keyboardState[SDL_SCANCODE_LCTRL] || keyboardState[SDL_SCANCODE_RCTRL];
        }
        
        // --- Find Snap Point for current frame ---
        int mouseX, mouseY;
        SDL_GetMouseState(&mouseX, &mouseY);
        Urbaxio::SnapResult currentSnap;
        if (toolManager.ShouldEnableSnapping()) {
            currentSnap = snappingSystem.FindSnapPoint(mouseX, mouseY, display_w, display_h, camera, *scene_ptr);
        } else {
            currentSnap = {};
            currentSnap.snapped = false;
        }
        
        // --- Process Input (always, for ImGui and quitting) ---
        inputHandler.ProcessEvents(camera, should_quit, window, toolManager, scene_ptr);

        // --- Update Active Tool ---
        toolManager.OnUpdate(currentSnap);
        
        // --- GPU Upload for new objects ---
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

        // --- NEW: Render Distance Text (for desktop mirror view) ---
        if (vr_mode && vrManager && vrManager->zoomTextAlpha > 0.01f) {
            // Use left-eye matrices for projection to desktop mirror
            const auto& vr_views = vrManager->GetViews();
            if (!vr_views.empty()) {
                glm::vec3 midpoint_world = vrManager->zoomMidPoint;
                float distance_world = vrManager->zoomDistance;

                // Transform midpoint by current world transform
                glm::vec3 midpoint_transformed = TransformPoint(vrManager->GetWorldTransform(), midpoint_world);

                glm::vec2 screenPos;
                if (Urbaxio::SnappingSystem::WorldToScreen(
                        midpoint_transformed,
                        vr_views[0].viewMatrix,
                        vr_views[0].projectionMatrix,
                        display_w, display_h,
                        screenPos))
                {
                    // Apply world scale to get virtual distance
                    float worldScale = glm::length(glm::vec3(vrManager->GetWorldTransform()[0]));
                    float distance_virtual = distance_world * worldScale;

                    std::string distStr;
                    if (distance_virtual > 1000.0f) {
                        distStr = fmt::format("{:.2f} km", distance_virtual / 1000.0f);
                    } else if (distance_virtual >= 1.0f) {
                        distStr = fmt::format("{:.2f} m", distance_virtual);
                    } else {
                        distStr = fmt::format("{:.0f} mm", distance_virtual * 1000.0f);
                    }

                    ImDrawList* drawList = ImGui::GetForegroundDrawList();
                    ImVec2 textSize = ImGui::CalcTextSize(distStr.c_str());
                    ImVec2 textPos = ImVec2(screenPos.x - textSize.x * 0.5f, screenPos.y - textSize.y * 0.5f);

                    drawList->AddRectFilled(
                        ImVec2(textPos.x - 5, textPos.y - 2),
                        ImVec2(textPos.x + textSize.x + 5, textPos.y + textSize.y + 2),
                        IM_COL32(0, 0, 0, (int)(128.0f * vrManager->zoomTextAlpha)),
                        3.0f);

                    drawList->AddText(
                        textPos,
                        IM_COL32(255, 255, 255, (int)(255.0f * vrManager->zoomTextAlpha)),
                        distStr.c_str());
                }
            }
        }

        // --- Main Controls Window ---
        { 
            ImGui::Begin("Urbaxio Controls");
            ImGui::Text("App avg %.3f ms/f (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
            ImGui::Separator();
            if (ImGui::Button("Create Box Object")) { object_counter++; std::string box_name = "Box_" + std::to_string(object_counter); scene_ptr->create_box_object(box_name, 10.0, 20.0, 5.0); }
            ImGui::Separator();
            
            if (ImGui::Button("Appearance Settings")) show_style_editor = true;
            
            ImGui::Separator();
            ImGui::Text("Tools:");
            Urbaxio::Tools::ToolType activeToolType = toolManager.GetActiveToolType();

            bool isSelect = activeToolType == Urbaxio::Tools::ToolType::Select;
            if (ImGui::RadioButton("Select", isSelect)) toolManager.SetTool(Urbaxio::Tools::ToolType::Select);
            ImGui::SameLine();
            bool isLine = activeToolType == Urbaxio::Tools::ToolType::Line;
            if (ImGui::RadioButton("Line", isLine)) toolManager.SetTool(Urbaxio::Tools::ToolType::Line);
            ImGui::SameLine();
            bool isMove = activeToolType == Urbaxio::Tools::ToolType::Move;
            if (ImGui::RadioButton("Move", isMove)) toolManager.SetTool(Urbaxio::Tools::ToolType::Move);
            ImGui::SameLine();
            bool isPushPull = activeToolType == Urbaxio::Tools::ToolType::PushPull;
            if (ImGui::RadioButton("Push/Pull", isPushPull)) toolManager.SetTool(Urbaxio::Tools::ToolType::PushPull);

            if (ImGui::Button("Clear Lines") && scene_ptr) { 
                scene_ptr->ClearUserLines(); 
                selectedLineIDs.clear();
                toolManager.SetTool(Urbaxio::Tools::ToolType::Line); // Reactivate to reset state
            }
            ImGui::SameLine();
            if (ImGui::Button("Run Split Test")) {
                if (scene_ptr) {
                    std::vector<Urbaxio::Engine::SceneObject*> objects_to_clean = scene_ptr->get_all_objects();
                    for (auto* obj : objects_to_clean) {
                        if (obj) {
                             if (obj->vao != 0) { glDeleteVertexArrays(1, &obj->vao); obj->vao = 0; }
                             if (obj->vbo_vertices != 0) { glDeleteBuffers(1, &obj->vbo_vertices); obj->vbo_vertices = 0; }
                             if (obj->vbo_normals != 0) { glDeleteBuffers(1, &obj->vbo_normals); obj->vbo_normals = 0; }
                             if (obj->ebo != 0) { glDeleteBuffers(1, &obj->ebo); obj->ebo = 0; }
                        }
                    }
                    scene_ptr->TestFaceSplitting(); 
                }
            }
            
            // --- Tool-specific UI ---
            toolManager.RenderUI();

            ImGui::Separator();
            if (ImGui::CollapsingHeader("VR Panel Debug")) {
                if (numpadInitialized) {
                    glm::vec3 scale;
                    glm::quat rotation;
                    glm::vec3 translation;
                    glm::vec3 skew;
                    glm::vec4 perspective;
                    glm::decompose(numpadOffsetTransform, scale, rotation, translation, skew, perspective);

                    glm::vec3 eulerAngles = glm::eulerAngles(rotation);
                    eulerAngles = glm::degrees(eulerAngles);
                    ImGui::InputFloat3("Translation", &translation.x, "%.3f");
                    ImGui::InputFloat3("Rotation (Euler)", &eulerAngles.x, "%.3f");
                    ImGui::InputFloat("Scale", &scale.x, 0.01f, 0.1f, "%.3f");
                    if (ImGui::Button("Apply Debug Values")) {
                        glm::mat4 newScale = glm::scale(glm::mat4(1.0f), glm::vec3(scale.x));
                        glm::mat4 newRotation = glm::mat4_cast(glm::quat(glm::radians(eulerAngles)));
                        glm::mat4 newTranslation = glm::translate(glm::mat4(1.0f), translation);
                        numpadOffsetTransform = newTranslation * newRotation * newScale;
                    }
                } else {
                    ImGui::TextDisabled("Numpad not yet initialized in VR.");
                }
            }

            ImGui::Text("Scene Info:");
            ImGui::Text("Lines in Scene: %zu", scene_ptr ? scene_ptr->GetAllLines().size() : 0);
            ImGui::Text("Selected Object ID: %llu", selectedObjId);
            if (selectedObjId != 0) ImGui::Text("Selected Triangles: %zu", selectedTriangleIndices.size());
            if (!selectedLineIDs.empty()) ImGui::Text("Selected Line IDs: %zu", selectedLineIDs.size());

            const char* snapTypeName = "None"; if (currentSnap.snapped) { switch (currentSnap.type) { case Urbaxio::SnapType::ENDPOINT: snapTypeName = "Endpoint"; break; case Urbaxio::SnapType::ORIGIN:   snapTypeName = "Origin"; break; case Urbaxio::SnapType::AXIS_X:   snapTypeName = "On Axis X"; break; case Urbaxio::SnapType::AXIS_Y:   snapTypeName = "On Axis Y"; break; case Urbaxio::SnapType::AXIS_Z:   snapTypeName = "On Axis Z"; break; default: snapTypeName = "Unknown"; break; } } ImGui::Text("Current Snap: %s (%.2f, %.2f, %.2f)", snapTypeName, currentSnap.worldPoint.x, currentSnap.worldPoint.y, currentSnap.worldPoint.z);
            ImGui::Text("Scene Objects:"); if (scene_ptr) { std::vector<Urbaxio::Engine::SceneObject*> objects = scene_ptr->get_all_objects(); if (objects.empty()) { ImGui::TextDisabled("(No objects yet)"); } else { ImGui::BeginChild("ObjectList", ImVec2(0, 100), true, ImGuiWindowFlags_HorizontalScrollbar); for (const auto* obj : objects) { if (obj) { ImGui::BulletText("%s (ID:%llu)%s%s%s", obj->get_name().c_str(), obj->get_id(), obj->has_shape() ? " [Geo]" : "", obj->has_mesh() ? " [Mesh]" : "", (obj->vao != 0) ? " [GPU]" : ""); } } ImGui::EndChild(); } } else { ImGui::TextDisabled("(Scene pointer is null)"); }
            ImGui::End();
        }

        // --- Appearance Settings Window ---
        if (show_style_editor) {
            ImGui::Begin("Appearance Settings", &show_style_editor);
            if (ImGui::CollapsingHeader("Scene Colors")) { ImGui::ColorEdit3("Background", (float*)&clear_color); ImGui::ColorEdit3("Default Object", (float*)&objectColor); }
            if (ImGui::CollapsingHeader("Lighting")) { 
                ImGui::TextDisabled("Light follows camera direction (headlamp mode)");
                ImGui::SliderFloat("Ambient Strength", &ambientStrength, 0.0f, 1.0f); 
                ImGui::ColorEdit3("Light Color", glm::value_ptr(lightColor)); 
            }
            if (ImGui::CollapsingHeader("Grid & Axes")) { ImGui::Checkbox("Show Grid", &showGrid); ImGui::SameLine(); ImGui::Checkbox("Show Axes", &showAxes); ImGui::ColorEdit3("Grid Color", glm::value_ptr(gridColor)); ImGui::SeparatorText("Positive Axes"); ImGui::ColorEdit4("Axis X Color", glm::value_ptr(axisColorX)); ImGui::ColorEdit4("Axis Y Color", glm::value_ptr(axisColorY)); ImGui::ColorEdit4("Axis Z Color", glm::value_ptr(axisColorZ)); ImGui::ColorEdit4("Fade To Color##Positive", glm::value_ptr(positiveAxisFadeColor)); ImGui::SameLine(); ImGui::TextDisabled("(also used for axis markers)"); ImGui::SliderFloat("Width##Positive", &axisLineWidth, 1.0f, maxLineWidth); ImGui::SeparatorText("Negative Axes"); ImGui::ColorEdit4("Fade To Color##Negative", glm::value_ptr(negativeAxisFadeColor)); ImGui::SliderFloat("Width##Negative", &negAxisLineWidth, 1.0f, maxLineWidth); }
            if (ImGui::CollapsingHeader("Interactive Effects")) { ImGui::SliderFloat("Cursor Radius", &cursorRadius, 1.0f, 50.0f); ImGui::SliderFloat("Effect Intensity", &effectIntensity, 0.1f, 2.0f); }
            
            if (ImGui::CollapsingHeader("Markers")) {
                bool radius_changed = ImGui::SliderFloat("Capsule Radius", &capsuleRadius, 0.1f, 2.0f);
                bool height10m_changed = ImGui::SliderFloat("10m Capsule Height", &capsuleHeight10m, 0.2f, 5.0f);
                bool height5m_changed = ImGui::SliderFloat("5m Capsule Height", &capsuleHeight5m, 0.2f, 5.0f);

                if (radius_changed || height10m_changed) {
                    Urbaxio::Engine::SceneObject* marker = scene_ptr->get_object_by_id(capsule_marker_10m->get_id());
                    if (marker) {
                        marker->set_mesh_buffers(CreateCapsuleMesh(capsuleRadius, capsuleHeight10m));
                        marker->vao = 0; // Invalidate GPU resource to trigger re-upload
                    }
                }
                if (radius_changed || height5m_changed) {
                    Urbaxio::Engine::SceneObject* marker = scene_ptr->get_object_by_id(capsule_marker_5m->get_id());
                     if (marker) {
                        marker->set_mesh_buffers(CreateCapsuleMesh(capsuleRadius, capsuleHeight5m));
                        marker->vao = 0; // Invalidate GPU resource to trigger re-upload
                    }
                }
            }
            ImGui::End();
        }

        ImGui::Render();

        if (vr_mode && vrManager->IsInitialized()) {
            // --- VR RENDER PATH ---
            uint32_t leftEyeImageIndex = -1; // For mirror view

            if (vrManager->BeginFrame()) {
                // Poll controller state after syncing actions in BeginFrame
                vrManager->PollActions();
                // Update modifier state for VR after polling actions
                shiftDown = vrManager->aButtonIsPressed;
                ctrlDown = false;
                // Prepare VR snap for this frame (used per-eye)
                Urbaxio::SnapResult vrSnap; vrSnap.snapped = false;
                // Ray used for both pointer and VR menu hit tests
                glm::vec3 vrRayOrigin(0.0f);
                glm::vec3 vrRayDirection(0.0f, 0.0f, -1.0f);

                // --- Update VR Pointer Ray with ergonomic tilt and snapping ---
                const auto& rightHand = vrManager->GetRightHandVisual();
                if (rightHand.isValid) {
                    glm::mat4 rawPoseMatrix = XrPoseToModelMatrix(rightHand.pose);
                    glm::mat4 finalPoseMatrix = vrManager->GetWorldTransform() * rawPoseMatrix;
                    vrRayOrigin = glm::vec3(finalPoseMatrix[3]);
                    // Tilt -65 degrees forward around local X, then use local -Z
                    glm::quat tilt = glm::angleAxis(glm::radians(-65.0f), glm::vec3(1.0f, 0.0f, 0.0f));
                    glm::vec3 localDir = tilt * glm::vec3(0.0f, 0.0f, -1.0f);
                    vrRayDirection = glm::normalize(glm::vec3(finalPoseMatrix * glm::vec4(localDir, 0.0f)));
                    // Dynamic, scale-aware snap radius
                    const float BASE_VR_SNAP_RADIUS = 0.01f;
                    float worldScale = glm::length(glm::vec3(vrManager->GetWorldTransform()[0]));
                    float dynamicSnapRadius = BASE_VR_SNAP_RADIUS * worldScale;
                    vrSnap = snappingSystem.FindSnapPointFromRay(vrRayOrigin, vrRayDirection, *scene_ptr, dynamicSnapRadius);
                } else {
                    renderer.UpdateVRPointer({}, {}, false);
                }

                glDisable(GL_FRAMEBUFFER_SRGB); // Use linear color space for rendering
                
                // --- REVISED: Prepare override maps for dynamic objects ---
                std::map<uint64_t, glm::mat4> transformOverrides;
                std::map<uint64_t, glm::vec3> colorOverrides;
                std::map<uint64_t, bool> unlitOverrides;
                
                // Get the cumulative world transform from the grab/zoom action
                const glm::mat4& worldTransform = vrManager->GetWorldTransform();

                const auto& leftHand = vrManager->GetLeftHandVisual();
                if (leftHand.isValid && leftControllerVisual) {
                    // Apply the world transform to the raw controller pose
                    glm::mat4 rawPoseMatrix = XrPoseToModelMatrix(leftHand.pose);
                    // REMOVED scale compensation. The controller now scales with the world.
                    transformOverrides[leftControllerVisual->get_id()] = worldTransform * rawPoseMatrix * glm::scale(glm::mat4(1.0f), glm::vec3(0.06f));
                    colorOverrides[leftControllerVisual->get_id()] = MixColorFromPress(leftHand.pressValue);
                    unlitOverrides[leftControllerVisual->get_id()] = true;
                }

                if (rightHand.isValid && rightControllerVisual) {
                    // Apply the world transform to the raw controller pose
                    glm::mat4 rawPoseMatrix = XrPoseToModelMatrix(rightHand.pose);
                    // REMOVED scale compensation. The controller now scales with the world.
                    transformOverrides[rightControllerVisual->get_id()] = worldTransform * rawPoseMatrix * glm::scale(glm::mat4(1.0f), glm::vec3(0.06f));
                    colorOverrides[rightControllerVisual->get_id()] = MixColorFromPress(rightHand.pressValue);
                    unlitOverrides[rightControllerVisual->get_id()] = true;
                }

                // Update VR pointer with menu clipping
                if (rightHand.isValid) {
                    float rayLength = 100.0f;
                    glm::vec3 rayEnd = (vrSnap.snapped && vrSnap.type != Urbaxio::SnapType::GRID) 
                                       ? vrSnap.worldPoint 
                                       : vrRayOrigin + vrRayDirection * rayLength;
                    if (vrManager->leftMenuAlpha > 0.01f && leftControllerVisual) {
                        float closestHit = std::numeric_limits<float>::max();
                        auto it = transformOverrides.find(leftControllerVisual->get_id());
                         if (it != transformOverrides.end()) {
                            const glm::mat4& controllerTransform = it->second;
                            float worldScale = glm::length(glm::vec3(vrManager->GetWorldTransform()[0]));
                            glm::vec3 controllerRight = glm::normalize(glm::vec3(controllerTransform[0]));
                            glm::quat menuTilt = glm::angleAxis(glm::radians(-70.0f), controllerRight);
                            glm::vec3 menuUp = menuTilt * glm::normalize(glm::vec3(controllerTransform[1]));
                            glm::vec3 menuStartPos = glm::vec3(controllerTransform[3]) + controllerRight * (0.1f * worldScale);
                            float lineSpacing = 0.05f * worldScale;
                            float sphereRadius = 0.02f * worldScale;
                            for (size_t tool_idx = 0; tool_idx < 4; ++tool_idx) {
                                glm::vec3 sphereCenter = menuStartPos - menuUp * (float)tool_idx * lineSpacing - controllerRight * (sphereRadius * 2.0f);
                                float dist;
                                if (glm::intersectRaySphere(vrRayOrigin, vrRayDirection, sphereCenter, sphereRadius * sphereRadius, dist) && dist < closestHit) {
                                    closestHit = dist;
                                }
                            }
                         }
                        if (closestHit < std::numeric_limits<float>::max() && closestHit < glm::distance(vrRayOrigin, rayEnd)) {
                            rayEnd = vrRayOrigin + vrRayDirection * closestHit;
                        }
                    }
                    renderer.UpdateVRPointer(vrRayOrigin, rayEnd, true);
                }

                // --- VR Tool Interaction Logic ---
                if (toolManager.GetActiveTool()) {
                    if (toolManager.GetActiveToolType() == Urbaxio::Tools::ToolType::PushPull) {
                        static_cast<Urbaxio::Tools::PushPullTool*>(toolManager.GetActiveTool())->updateHover(vrRayOrigin, vrRayDirection);
                    }
                    toolManager.GetActiveTool()->OnUpdate(vrSnap, vrRayOrigin, vrRayDirection);
                }
                
                if (rightHand.triggerClicked) {
                    if (isHoveringGrabHandle) {
                        isGrabbingNumpad = true;
                        grabbedNumpadInitialTransform = numpadTransform;
                        grabbedControllerInitialTransform = vrManager->GetWorldTransform() * XrPoseToModelMatrix(rightHand.pose);
                    } else {
                    toolManager.OnLeftMouseDown(0, 0, *toolContext.shiftDown, *toolContext.ctrlDown, vrRayOrigin, vrRayDirection);
                    }
                }
                if (rightHand.triggerReleased) {
                    if (isGrabbingNumpad) {
                        isGrabbingNumpad = false;
                        // When released, calculate the new relative offset from the controller
                        auto it = transformOverrides.find(leftControllerVisual->get_id());
                        if (it != transformOverrides.end()) {
                            const glm::mat4& controllerTransform = it->second;
                            numpadOffsetTransform = glm::inverse(controllerTransform) * numpadTransform;
                        }
                    }
                    toolManager.OnLeftMouseUp(0, 0, *toolContext.shiftDown, *toolContext.ctrlDown);
                }

                const auto& vr_views = vrManager->GetViews();
                for (uint32_t i = 0; i < vr_views.size(); ++i) {
                    const auto& swapchain = vrManager->GetSwapchain(i);
                    uint32_t imageIndex = vrManager->AcquireSwapchainImage(i);
                    if (i == 0) { leftEyeImageIndex = imageIndex; }
                    
                    // Bind pre-created FBO for this swapchain image
                    glBindFramebuffer(GL_FRAMEBUFFER, swapchain.fbos[imageIndex]);
                    glViewport(0, 0, swapchain.width, swapchain.height);
                    
                    // Clear this eye's buffer
                    glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
                    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

                    // Get view-specific data
                    const auto& current_view = vr_views[i];
                    const glm::mat4& view = current_view.viewMatrix;
                    const glm::mat4& projection = current_view.projectionMatrix;
                    const glm::vec3 viewPos = glm::vec3(glm::inverse(view)[3]);
                    
                    uint64_t previewObjId = 0;
                    if (toolManager.GetActiveToolType() == Urbaxio::Tools::ToolType::Move) {
                        auto* moveTool = static_cast<Urbaxio::Tools::MoveTool*>(toolManager.GetActiveTool());
                        previewObjId = moveTool->GetMovingObjectId();
                    }
                    renderer.UpdateUserLinesBuffer(scene_ptr->GetAllLines(), *toolContext.selectedLineIDs, previewObjId, scene_ptr);
                    toolManager.RenderPreview(renderer, vrSnap);
                    
                    // Call the main render function BEFORE drawing UI elements like menus
                    renderer.RenderFrame(
                        swapchain.width, swapchain.height,
                        view, projection, viewPos, scene_ptr,
                        objectColor, lightColor, ambientStrength, 
                        showGrid, showAxes, axisLineWidth, negAxisLineWidth,
                        gridColor, axisColorX, axisColorY, axisColorZ, positiveAxisFadeColor, negativeAxisFadeColor,
                        vrSnap.worldPoint, cursorRadius, effectIntensity, 
                        *toolContext.selectedObjId, *toolContext.selectedTriangleIndices, *toolContext.selectedLineIDs, selectionHighlightColor, 
                        *toolContext.hoveredObjId, *toolContext.hoveredFaceTriangleIndices, hoverHighlightColor,
                        vrSnap,
                        nullptr, // No ImGui data for VR eyes
                        previewObjId, glm::mat4(1.0f),
                        transformOverrides,
                        colorOverrides,
                        unlitOverrides
                    );
                    
                    // --- Render VR Tool Menu ---
                    if (vrManager->leftMenuAlpha > 0.01f && leftControllerVisual) {
                        auto it = transformOverrides.find(leftControllerVisual->get_id());
                        if (it != transformOverrides.end()) {
                            const glm::mat4& controllerTransform = it->second;
                            float worldScale = glm::length(glm::vec3(vrManager->GetWorldTransform()[0]));

                            glm::vec3 controllerPos = glm::vec3(controllerTransform[3]);
                            glm::vec3 controllerRight = glm::normalize(glm::vec3(controllerTransform[0]));
                            glm::vec3 controllerUp = glm::normalize(glm::vec3(controllerTransform[1]));
                            // Apply a -70-degree tilt around the RIGHT axis (tablet-like tilt)
                            glm::quat menuTilt = glm::angleAxis(glm::radians(-70.0f), controllerRight);
                            glm::vec3 menuUp = menuTilt * controllerUp;

                            // Offset and spacing scale with worldScale to keep menu attached to the controller
                            glm::vec3 menuStartPos = controllerPos + controllerRight * (0.1f * worldScale);
                            float lineSpacing = 0.05f * worldScale;

                            const std::vector<std::string> toolNames = {"Select", "Line", "Push/Pull", "Move"};
                            if (toolMenuAlphas.size() != toolNames.size()) {
                                toolMenuAlphas.assign(toolNames.size(), 0.0f);
                            }

                            // Ray-menu hit testing
                            hoveredToolIndex = -1;
                            float closestHit = std::numeric_limits<float>::max();
                            float hitboxHeight = 0.04f * worldScale;
                            float hitboxWidth = 0.2f * worldScale;
                            
                            if (rightHand.isValid) {
                                for (size_t tool_idx = 0; tool_idx < toolNames.size(); ++tool_idx) {
                                    glm::vec3 itemTextPos = menuStartPos - menuUp * (float)tool_idx * lineSpacing;
                                    
                                    glm::vec3 p0 = itemTextPos - controllerRight * (hitboxWidth / 2.0f) + menuUp * (hitboxHeight / 2.0f);
                                    glm::vec3 p1 = itemTextPos + controllerRight * (hitboxWidth / 2.0f) + menuUp * (hitboxHeight / 2.0f);
                                    glm::vec3 p2 = itemTextPos + controllerRight * (hitboxWidth / 2.0f) - menuUp * (hitboxHeight / 2.0f);
                                    glm::vec3 p3 = itemTextPos - controllerRight * (hitboxWidth / 2.0f) - menuUp * (hitboxHeight / 2.0f);
                                    glm::vec2 baryPosition;
                                    float dist1, dist2;
                                    bool hit1 = glm::intersectRayTriangle(vrRayOrigin, vrRayDirection, p0, p1, p2, baryPosition, dist1);
                                    bool hit2 = glm::intersectRayTriangle(vrRayOrigin, vrRayDirection, p0, p2, p3, baryPosition, dist2);
                                    if ((hit1 && dist1 < closestHit) || (hit2 && dist2 < closestHit)) {
                                        closestHit = hit1 ? dist1 : dist2;
                                        hoveredToolIndex = static_cast<int>(tool_idx);
                                    }
                                }
                            }

                            // --- Tool Selection Logic ---
                            if (rightHand.triggerClicked && hoveredToolIndex != -1) {
                                Urbaxio::Tools::ToolType selectedToolType;
                                switch (hoveredToolIndex) {
                                    case 0: selectedToolType = Urbaxio::Tools::ToolType::Select; break;
                                    case 1: selectedToolType = Urbaxio::Tools::ToolType::Line; break;
                                    case 2: selectedToolType = Urbaxio::Tools::ToolType::PushPull; break;
                                    case 3: selectedToolType = Urbaxio::Tools::ToolType::Move; break;
                                    default: selectedToolType = Urbaxio::Tools::ToolType::Select; break;
                                }
                                toolManager.SetTool(selectedToolType);
                            }

                            const float HIGHLIGHT_FADE_SPEED = 0.15f;
                            Urbaxio::Tools::ToolType activeToolType = toolManager.GetActiveToolType();
                            for (size_t tool_idx = 0; tool_idx < toolNames.size(); ++tool_idx) {
                                Urbaxio::Tools::ToolType toolType;
                                switch (tool_idx) {
                                    case 0: toolType = Urbaxio::Tools::ToolType::Select; break;
                                    case 1: toolType = Urbaxio::Tools::ToolType::Line; break;
                                    case 2: toolType = Urbaxio::Tools::ToolType::PushPull; break;
                                    case 3: toolType = Urbaxio::Tools::ToolType::Move; break;
                                    default: toolType = Urbaxio::Tools::ToolType::Select; break;
                                }
                                bool isSelected = activeToolType == toolType;
                                bool isHovered = hoveredToolIndex == static_cast<int>(tool_idx);
                                
                                float targetAlpha = isHovered ? 1.0f : (isSelected ? 0.8f : 0.5f);
                                toolMenuAlphas[tool_idx] += (targetAlpha - toolMenuAlphas[tool_idx]) * HIGHLIGHT_FADE_SPEED;
                                glm::vec3 itemTextPos = menuStartPos - menuUp * (float)tool_idx * lineSpacing;

                                // Render Sprite Widget (Quad)
                                float widget_diameter_scaled = 0.03f * worldScale;
                                glm::vec3 widgetCenter = itemTextPos - controllerRight * (widget_diameter_scaled * 0.75f + 0.01f * worldScale);
                                
                                // --- Build Model Matrix for the Billboard ---
                                // 1. Get camera's world-space orientation vectors from the inverse view matrix.
                                // These vectors are guaranteed to be orthogonal and define the camera's orientation.
                                glm::mat4 cameraWorld = glm::inverse(view);
                                glm::vec3 camRight = glm::normalize(glm::vec3(cameraWorld[0]));
                                glm::vec3 camUp    = glm::normalize(glm::vec3(cameraWorld[1]));
                                glm::vec3 camFwd   = glm::normalize(glm::vec3(cameraWorld[2]));
                                
                                // 2. Construct the model matrix manually.
                                // The rotation part aligns the widget with the camera, the scale part sizes it,
                                // and the translation part positions it.
                                glm::mat4 finalModel = glm::translate(glm::mat4(1.0f), widgetCenter) * // 3. Translate
                                                       glm::mat4(glm::mat3(camRight, camUp, camFwd)) * // 2. Rotate to face camera
                                                       glm::scale(glm::mat4(1.0f), glm::vec3(widget_diameter_scaled)); // 1. Scale
                                // --- End Build Model Matrix ---
                                
                                // Define colors
                                glm::vec3 selectedColor = glm::vec3(1.0f, 0.79f, 0.4f);
                                glm::vec3 inactiveColor = glm::vec3(0.3f, 0.75f, 1.0f);
                                
                                // Define aberration colors
                                glm::vec3 orange_aberration1 = glm::vec3(1.00f, 0.84f, 0.26f);
                                glm::vec3 orange_aberration2 = glm::vec3(1.0f, 0.1f, 0.1f);
                                glm::vec3 blue_aberration1 = glm::vec3(0.67f, 0.5f, 1.0f);
                                glm::vec3 blue_aberration2 = glm::vec3(0.3f, 1.0f, 0.76f);
                                
                                // Smoothly interpolate all colors based on selection state
                                float selectionFactor = isSelected ? 1.0f : 0.0f;
                                glm::vec3 baseColor = glm::mix(inactiveColor, selectedColor, selectionFactor);
                                glm::vec3 abColor1 = glm::mix(blue_aberration1, orange_aberration1, selectionFactor);
                                glm::vec3 abColor2 = glm::mix(blue_aberration2, orange_aberration2, selectionFactor);
                                
                                // Aberration amount still animates on hover
                                float aberration = toolMenuAlphas[tool_idx] * 0.11f;
                                
                                renderer.RenderVRMenuWidget(
                                    view, projection,
                                    finalModel,
                                    baseColor, aberration, vrManager->leftMenuAlpha,
                                    abColor1, abColor2
                                );
                                
                                // Render Text
                                const float desiredPxHeight = 40.0f; 
                                float viewportH = static_cast<float>(swapchain.height);
                                const auto& fov = current_view.fov;
                                float viewZ = -(view * glm::vec4(itemTextPos, 1.0f)).z;
                                viewZ = std::max(0.1f, viewZ);
                                float worldUnitsPerPixel = viewZ * (tanf(fov.angleUp) - tanf(fov.angleDown)) / viewportH;
                                float textWorldSize = desiredPxHeight * worldUnitsPerPixel;
                                float finalWorldHeight = textWorldSize * worldScale;

                                textRenderer.AddText(
                                    toolNames[tool_idx],
                                    itemTextPos,
                                    glm::vec4(1.0f, 1.0f, 1.0f, vrManager->leftMenuAlpha * toolMenuAlphas[tool_idx]),
                                    finalWorldHeight,
                                    view
                                );
                            }
                        }
                    }

                    // --- Render VR Numpad ---
                    if (vrManager->leftMenuAlpha > 0.01f && leftControllerVisual) {
                        auto it = transformOverrides.find(leftControllerVisual->get_id());
                        if (it != transformOverrides.end()) {
                            const glm::mat4& controllerTransform = it->second;

                            float worldScale = glm::length(glm::vec3(vrManager->GetWorldTransform()[0]));

                            // --- Numpad Transform Logic ---
                            if (isGrabbingNumpad) {
                                if (rightHand.isValid) {
                                    glm::mat4 currentControllerTransform = vrManager->GetWorldTransform() * XrPoseToModelMatrix(rightHand.pose);
                                    glm::mat4 deltaTransform = currentControllerTransform * glm::inverse(grabbedControllerInitialTransform);
                                    numpadTransform = deltaTransform * grabbedNumpadInitialTransform;
                                }
                            } else {
                                // If not grabbed, the panel's transform is based on the controller's transform plus a relative offset.
                                if (!numpadInitialized) {
                                    // On first appearance, calculate the default offset from the new tuned values
                                    float panelScale = 8.0f;
                                    glm::vec3 translation = glm::vec3(-0.007f, -0.584f, -2.276f);
                                    glm::vec3 eulerAnglesRad = glm::radians(glm::vec3(-104.560f, 1.023f, -11.718f));

                                    numpadOffsetTransform = glm::translate(glm::mat4(1.0f), translation) *
                                                          glm::mat4_cast(glm::quat(eulerAnglesRad)) *
                                                          glm::scale(glm::mat4(1.0f), glm::vec3(panelScale));
                                    numpadInitialized = true;
                                }
                                numpadTransform = controllerTransform * numpadOffsetTransform;
                            }

                            glm::vec3 numpadOrigin = glm::vec3(numpadTransform[3]);
                            glm::vec3 numpadRight = glm::normalize(glm::vec3(numpadTransform[0]));
                            glm::vec3 numpadUp = glm::normalize(glm::vec3(numpadTransform[1]));

                            float panelLocalScale = glm::length(glm::vec3(numpadTransform[0])); // Extract scale from the matrix

                            // --- Display & Grab Handle ---
                            float displayWidth = 0.2f * panelLocalScale;
                            float displayHeight = 0.05f * panelLocalScale;
                            glm::vec3 displayCenter = numpadOrigin + numpadUp * (0.06f * panelLocalScale);
                            
                            // --- Update Numpad Display String ---
                            if (toolManager.GetActiveToolType() == Urbaxio::Tools::ToolType::Line) {
                                auto* lineTool = static_cast<Urbaxio::Tools::LineTool*>(toolManager.GetActiveTool());
                                if (lineTool->IsDrawing()) {
                                    float length_m = lineTool->GetCurrentLineLength();
                                    numpadInput = fmt::format("{:.0f}", length_m * 1000.0f);
                                }
                            } else {
                                // Clear display if another tool is active
                                numpadInput = "0";
                            }
                            
                            // Render the display panel
                            glm::mat4 displayModel = glm::translate(glm::mat4(1.0f), displayCenter) *
                                                     glm::mat4(glm::mat3(numpadRight, numpadUp, glm::cross(numpadRight, numpadUp))) *
                                                     glm::scale(glm::mat4(1.0f), glm::vec3(displayWidth, displayHeight, 1.0f));
                            renderer.RenderVRPanel(view, projection, displayModel, glm::vec3(0.1f, 0.15f, 0.25f), 0.2f, 0.7f * vrManager->leftMenuAlpha);
                            
                            // Render text on top of the display using the new method
                            // We create a transform for the text itself, positioned at the display center
                            glm::mat4 textTransform = glm::translate(glm::mat4(1.0f), displayCenter) *
                                                      glm::mat4(glm::mat3(numpadRight, numpadUp, glm::cross(numpadRight, numpadUp)));
                            
                            textRenderer.SetPanelModelMatrix(textTransform);
                            textRenderer.AddTextOnPanel(numpadInput, glm::mat4(1.0f), glm::vec4(1.0f), 0.03f * panelLocalScale);
                            float grabHandleRadius = 0.012f * panelLocalScale;
                            glm::vec3 grabHandleCenter = displayCenter + numpadUp * (displayHeight * 0.5f + grabHandleRadius + 0.01f * panelLocalScale);

                            isHoveringGrabHandle = false;
                            if (rightHand.isValid) {
                                float dist;
                                if (glm::intersectRaySphere(vrRayOrigin, vrRayDirection, grabHandleCenter, grabHandleRadius * grabHandleRadius, dist)) {
                                    isHoveringGrabHandle = true;
                                }
                            }

                            // Build model matrix for billboard grab handle
                            glm::mat4 cameraWorld = glm::inverse(view);
                            glm::vec3 camRight = glm::normalize(glm::vec3(cameraWorld[0]));
                            glm::vec3 camUp    = glm::normalize(glm::vec3(cameraWorld[1]));
                            glm::vec3 camFwd   = glm::normalize(glm::vec3(cameraWorld[2]));

                            glm::mat4 grabHandleModel = glm::translate(glm::mat4(1.0f), grabHandleCenter) *
                                                   glm::mat4(glm::mat3(camRight, camUp, camFwd)) *
                                                   glm::scale(glm::mat4(1.0f), glm::vec3(grabHandleRadius * 2.0f));
                            renderer.RenderVRMenuWidget(view, projection, grabHandleModel, glm::vec3(1.0f), isHoveringGrabHandle ? 0.15f : 0.05f, vrManager->leftMenuAlpha, glm::vec3(1.0f), glm::vec3(1.0f));

                            // --- Buttons ---
                            const char* keys[12] = {"1","2","3", "4","5","6", "7","8","9", ".","0",""}; // last is confirm
                            float keySpacing = 0.06f * panelLocalScale;
                            float keyRadius = 0.025f * panelLocalScale;
                            hoveredNumpadKey = -1;
                            float closestHit = std::numeric_limits<float>::max();

                            // Exclude grab handle from key intersection test
                            if (isHoveringGrabHandle) {
                                closestHit = 0; // Prioritize grab handle
                            }

                            for (int i = 0; i < 12; ++i) {
                                int row = i / 3;
                                int col = i % 3;
                                glm::vec3 keyCenter = numpadOrigin -
                                                    numpadUp * (float)row * keySpacing +
                                                    numpadRight * ((float)col - 1.0f) * keySpacing;

                                if (i == 9) { // '.' key
                                    keyCenter = numpadOrigin - numpadUp * 3.f * keySpacing - numpadRight * 1.f * keySpacing;
                                } else if (i == 10) { // '0' key
                                    keyCenter = numpadOrigin - numpadUp * 3.f * keySpacing;
                                } else if (i == 11) { // Confirm key
                                    keyCenter = numpadOrigin - numpadUp * 3.f * keySpacing + numpadRight * 1.f * keySpacing;
                                }

                                // Intersection check
                                if (rightHand.isValid && !isHoveringGrabHandle) { // Don't check keys if hovering handle
                                    float dist;
                                    if (glm::intersectRaySphere(vrRayOrigin, vrRayDirection, keyCenter, keyRadius * keyRadius, dist) && dist < closestHit) {
                                        closestHit = dist;
                                        hoveredNumpadKey = i;
                                    }
                                }

                                bool isHovered = (hoveredNumpadKey == i);

                                // Render key (text or widget)
                                if (i < 11) { // Text keys
                                    textRenderer.AddText(
                                        keys[i],
                                        keyCenter,
                                        glm::vec4(1.0f, 1.0f, 1.0f, vrManager->leftMenuAlpha * (isHovered ? 1.0f : 0.7f)),
                                        0.03f * panelLocalScale,
                                        view
                                    );
                                } else { // Confirm key (widget)
                                    float widget_diameter = 0.03f * panelLocalScale;

                                    // Build model matrix for billboard
                                    glm::mat4 cameraWorld = glm::inverse(view);
                                    glm::vec3 camRight = glm::normalize(glm::vec3(cameraWorld[0]));
                                    glm::vec3 camUp    = glm::normalize(glm::vec3(cameraWorld[1]));
                                    glm::vec3 camFwd   = glm::normalize(glm::vec3(cameraWorld[2]));

                                    glm::mat4 finalModel = glm::translate(glm::mat4(1.0f), keyCenter) *
                                                           glm::mat4(glm::mat3(camRight, camUp, camFwd)) *
                                                           glm::scale(glm::mat4(1.0f), glm::vec3(widget_diameter));

                                    glm::vec3 baseColor = glm::vec3(0.1f, 0.8f, 0.2f); // Green
                                    float aberration = isHovered ? 0.15f : 0.05f;
                                    renderer.RenderVRMenuWidget(view, projection, finalModel, baseColor, aberration, vrManager->leftMenuAlpha, baseColor, baseColor);
                                }
                            }
                        }
                    }
                    
                    // 3D distance text in VR
                    if (vrManager->zoomTextAlpha > 0.01f) {
                        float worldScale = glm::length(glm::vec3(worldTransform[0]));
                        float distance_virtual = vrManager->zoomDistance * worldScale;
                        std::string distStr;
                        if (distance_virtual > 1000.0f) distStr = fmt::format("{:.2f} km", distance_virtual / 1000.0f);
                        else if (distance_virtual >= 1.0f) distStr = fmt::format("{:.2f} m", distance_virtual);
                        else distStr = fmt::format("{:.0f} mm", distance_virtual * 1000.0f);
                        glm::vec3 midpoint_transformed = TransformPoint(worldTransform, vrManager->zoomMidPoint);

                        // Keep visual size constant by deriving world height from FOV and depth,
                        // then invert by world scale (stronger effect)
                        const float desiredPx = 20.0f; // much stronger visual size
                        float viewportH = static_cast<float>(swapchain.height);
                        const auto& fov = current_view.fov;

                        float viewZ = -(view * glm::vec4(midpoint_transformed, 1.0f)).z;
                        viewZ = std::max(0.1f, viewZ);

                        float worldUnitsPerPixel = viewZ * (tanf(fov.angleUp) - tanf(fov.angleDown)) / viewportH;
                        float textWorldSize = desiredPx * worldUnitsPerPixel;
                        float finalWorldHeight = textWorldSize * worldScale;

                        const float textOpacity = 0.7f;
                        textRenderer.AddText(distStr, midpoint_transformed, glm::vec4(1,1,1, vrManager->zoomTextAlpha * textOpacity), finalWorldHeight, view);
                    }
                    textRenderer.Render(view, projection); // Now this renders ALL text (menu, numpad, distance)

                    glFinish();
                    glBindFramebuffer(GL_FRAMEBUFFER, 0);
                    
                    vrManager->ReleaseSwapchainImage(i);
                }

                vrManager->EndFrame();
            }

            // Render desktop mirror view - show left eye
            if (leftEyeImageIndex != -1) {
                const auto& leftSwapchain = vrManager->GetSwapchain(0);
                glBindFramebuffer(GL_READ_FRAMEBUFFER, leftSwapchain.fbos[leftEyeImageIndex]);
                glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0); // Default framebuffer (the window)
                glBlitFramebuffer(
                    0, 0, leftSwapchain.width, leftSwapchain.height,
                    0, 0, display_w, display_h,
                    GL_COLOR_BUFFER_BIT, GL_LINEAR
                );
                glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
            } else {
                // Fallback if no frame was rendered
                renderer.SetViewport(0, 0, display_w, display_h);
                glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            }

            // Render ImGui on top of the mirrored view
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        } else {
            // --- 2D RENDER PATH ---
            renderer.UpdateVRPointer({}, {}, false); // Ensure VR pointer is off
            int mouseX, mouseY;
            SDL_GetMouseState(&mouseX, &mouseY);
            glm::vec3 cursorWorldPos = currentSnap.snapped
                ? currentSnap.worldPoint
                : inputHandler.GetCursorPointInWorld(camera, mouseX, mouseY, display_w, display_h, glm::vec3(0.0f));

            renderer.SetViewport(0, 0, display_w, display_h);
            glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            uint64_t previewObjId = 0;
            const Urbaxio::CadKernel::MeshBuffers* ghostMesh = nullptr;
            const std::vector<unsigned int>* ghostWireframeIndices = nullptr;
            if (toolManager.GetActiveToolType() == Urbaxio::Tools::ToolType::Move) {
                auto* moveTool = static_cast<Urbaxio::Tools::MoveTool*>(toolManager.GetActiveTool());
                previewObjId = moveTool->GetMovingObjectId();
                ghostMesh = moveTool->GetGhostMesh();
                ghostWireframeIndices = moveTool->GetGhostWireframeIndices();
            }
            if (ghostMesh) { renderer.UpdateGhostMesh(*ghostMesh, ghostWireframeIndices ? *ghostWireframeIndices : std::vector<unsigned int>()); } 
            else { renderer.ClearGhostMesh(); }
            renderer.UpdateUserLinesBuffer(scene_ptr->GetAllLines(), selectedLineIDs, previewObjId, scene_ptr);

            toolManager.RenderPreview(renderer, currentSnap);
            
            // Get matrices and position from camera for the 2D view
            glm::mat4 view = camera.GetViewMatrix();
            glm::mat4 projection = camera.GetProjectionMatrix((float)display_w / (float)display_h);

            renderer.RenderFrame(
                display_w, display_h,
                view, projection, camera.Position,
                scene_ptr, 
                objectColor, lightColor, ambientStrength, 
                showGrid, showAxes, axisLineWidth, negAxisLineWidth,
                gridColor, axisColorX, axisColorY, axisColorZ, positiveAxisFadeColor, negativeAxisFadeColor,
                cursorWorldPos, cursorRadius, effectIntensity, 
                selectedObjId, selectedTriangleIndices, selectedLineIDs, selectionHighlightColor, 
                hoveredObjId, hoveredFaceTriangleIndices, hoverHighlightColor,
                currentSnap, 
                ImGui::GetDrawData(),
                previewObjId, glm::mat4(1.0f), {}, {}, {});

            if (toolManager.GetActiveToolType() == Urbaxio::Tools::ToolType::Select) {
                auto* selectTool = static_cast<Urbaxio::Tools::SelectTool*>(toolManager.GetActiveTool());
                if (selectTool->IsDragging()) {
                    glm::vec2 start, end;
                    selectTool->GetDragRect(start, end);
                    renderer.RenderSelectionBox(start, end, display_w, display_h);
                }
            }
        }
            
        SDL_GL_SwapWindow(window);
    }

    std::cout << "Shell: <<< Exiting main loop." << std::endl;
    std::cout << "Shell: Cleaning up..." << std::endl; 
    if (vrManager) vrManager->Shutdown();
    if (scene_ptr) { std::vector<Urbaxio::Engine::SceneObject*> objects_to_clean = scene_ptr->get_all_objects(); for (auto* obj : objects_to_clean) { if (obj && obj->vao != 0) glDeleteVertexArrays(1, &obj->vao); if (obj && obj->vbo_vertices != 0) glDeleteBuffers(1, &obj->vbo_vertices); if (obj && obj->vbo_normals != 0) glDeleteBuffers(1, &obj->vbo_normals); if (obj && obj->ebo != 0) glDeleteBuffers(1, &obj->ebo); } } ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplSDL2_Shutdown(); ImGui::DestroyContext(); SDL_GL_DeleteContext(gl_context); SDL_DestroyWindow(window); SDL_Quit(); std::cout << "Shell: Urbaxio Application finished gracefully." << std::endl; return 0;
}