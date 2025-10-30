// Force dGPU usage on hybrid laptops (NVIDIA/AMD)
extern "C" {
    __declspec(dllexport) unsigned long NvOptimusEnablement = 0x00000001;        // NVIDIA Optimus
    __declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;          // AMD PowerXpress
}

// --- Includes ---
#include <engine/engine.h>
#include <engine/scene.h>
#include <engine/scene_object.h>
#include <cad_kernel/cad_kernel.h>
#include <tools/ToolManager.h>
#include <tools/SelectTool.h>
#include <tools/MoveTool.h>

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
#include <glm/gtx/norm.hpp>

#include <fmt/core.h>

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <set>
#include <map>
#include <cstdint>

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

    Urbaxio::Tools::ToolContext toolContext;
    toolContext.scene = scene_ptr;
    toolContext.camera = &camera;
    toolContext.window = window;
    toolContext.display_w = &display_w;
    toolContext.display_h = &display_h;
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

    bool should_quit = false; std::cout << "Shell: >>> Entering main loop..." << std::endl;
    while (!should_quit) {
        SDL_GetWindowSize(window, &display_w, &display_h);
        
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
            if (vrManager->BeginFrame()) {
                glDisable(GL_FRAMEBUFFER_SRGB);
                
                const auto& vr_views = vrManager->GetViews();
                for (uint32_t i = 0; i < vr_views.size(); ++i) {
                    const auto& swapchain = vrManager->GetSwapchain(i);
                    uint32_t imageIndex = vrManager->AcquireSwapchainImage(i);
                    
                    // Bind pre-created FBO for this swapchain image
                    glBindFramebuffer(GL_FRAMEBUFFER, swapchain.fbos[imageIndex]);
                    glViewport(0, 0, swapchain.width, swapchain.height);
                    
                    // --- Stage 1 Test: Render different colors to each eye ---
                    if (i == 0) { 
                        glClearColor(1.0f, 0.0f, 0.0f, 1.0f); 
                    } else { 
                        glClearColor(0.0f, 0.0f, 1.0f, 1.0f); 
                    }
                    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                    // TODO: Render the actual scene here using vr_views[i] matrices
                    
                    glFinish();
                    glBindFramebuffer(GL_FRAMEBUFFER, 0);
                    
                    vrManager->ReleaseSwapchainImage(i);
                }
                vrManager->EndFrame();
            }

            // Render desktop mirror view - show left eye
            renderer.SetViewport(0, 0, display_w, display_h);
            glClearColor(1.0f, 0.0f, 0.0f, 1.0f); // Red to match left eye
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        } else {
            // --- 2D RENDER PATH ---
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

            int mouseX, mouseY;
            SDL_GetMouseState(&mouseX, &mouseY);
            glm::vec3 cursorWorldPos = currentSnap.snapped
                ? currentSnap.worldPoint
                : inputHandler.GetCursorPointInWorld(camera, mouseX, mouseY, display_w, display_h, glm::vec3(0.0f));

            renderer.SetViewport(0, 0, display_w, display_h);
            glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            toolManager.RenderPreview(renderer, currentSnap);
            
            renderer.RenderFrame(window, camera, scene_ptr, objectColor, lightColor, ambientStrength, 
                showGrid, showAxes, axisLineWidth, negAxisLineWidth,
                gridColor, axisColorX, axisColorY, axisColorZ, positiveAxisFadeColor, negativeAxisFadeColor,
                cursorWorldPos, cursorRadius, effectIntensity, 
                selectedObjId, selectedTriangleIndices, selectedLineIDs, selectionHighlightColor, 
                hoveredObjId, hoveredFaceTriangleIndices, hoverHighlightColor,
                currentSnap, 
                ImGui::GetDrawData(),
                previewObjId
            );

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