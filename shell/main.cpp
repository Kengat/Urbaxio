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
#include <SDL2/SDL_syswm.h>
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
#include "file_io.h"
#include <filesystem>
// --- NEW: VR UI System includes ---
#include "ui/IVRWidget.h"
#include "ui/VRButtonWidget.h"
#include "ui/VRConfirmButtonWidget.h"
#include "ui/VRMenuSphereWidget.h"
#include "ui/VRToolButtonWidget.h"
#include "ui/VRDisplayWidget.h"
#include "ui/VRUIManager.h"
#include "ui/VRPanel.h"
// --- VR menu interaction helpers ---
#include <glm/gtx/intersect.hpp>

#include <fstream>
#include <charconv>
// --- NEW: For File Dialogs on Windows ---
#if defined(_WIN32)
#include <windows.h>
#endif

#include <limits>
#include <cmath>
#include <map>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

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

    // --- IcoSphere moved from engine_main to be a shell responsibility ---
    // Helper function for icosphere subdivision
    int get_middle_point(int p1, int p2, std::map<long long, int>& middlePointIndexCache, std::vector<glm::vec3>& vertices, float radius) {
        bool firstIsSmaller = p1 < p2;
        long long smallerIndex = firstIsSmaller ? p1 : p2;
        long long greaterIndex = firstIsSmaller ? p2 : p1;
        long long key = (smallerIndex << 32) + greaterIndex;

        auto it = middlePointIndexCache.find(key);
        if (it != middlePointIndexCache.end()) {
            return it->second;
        }

        glm::vec3 point1 = vertices[p1];
        glm::vec3 point2 = vertices[p2];
        glm::vec3 middle = glm::vec3(
            (point1.x + point2.x) / 2.0f,
            (point1.y + point2.y) / 2.0f,
            (point1.z + point2.z) / 2.0f
        );
        
        vertices.push_back(glm::normalize(middle) * radius);
        int i = vertices.size() - 1;
        middlePointIndexCache[key] = i;
        return i;
    }

    // Creates an icosphere mesh with a specified subdivision level
    Urbaxio::CadKernel::MeshBuffers CreateIcoSphereMesh(float radius, int subdivision) {
        Urbaxio::CadKernel::MeshBuffers mesh;
        const float t = (1.0f + std::sqrt(5.0f)) / 2.0f;
        std::vector<glm::vec3> base_vertices = { {-1,  t,  0}, { 1,  t,  0}, {-1, -t,  0}, { 1, -t,  0}, { 0, -1,  t}, { 0,  1,  t}, { 0, -1, -t}, { 0,  1, -t}, { t,  0, -1}, { t,  0,  1}, {-t,  0, -1}, {-t,  0,  1} };
        for (auto& v : base_vertices) { v = glm::normalize(v) * radius; }
        std::vector<unsigned int> base_indices = { 0, 11,  5,    0,  5,  1,    0,  1,  7,    0,  7, 10,    0, 10, 11, 1,  5,  9,    5, 11,  4,   11, 10,  2,   10,  7,  6,    7,  1,  8, 3,  9,  4,    3,  4,  2,    3,  2,  6,    3,  6,  8,    3,  8,  9, 4,  9,  5,    2,  4, 11,    6,  2, 10,    8,  6,  7,    9,  8,  1 };
        std::map<long long, int> middlePointIndexCache;
        std::vector<glm::vec3> temp_vertices_glm = base_vertices;
        for (int s = 0; s < subdivision; s++) {
            std::vector<unsigned int> new_indices;
            for (size_t i = 0; i < base_indices.size(); i += 3) {
                int i0 = base_indices[i], i1 = base_indices[i+1], i2 = base_indices[i+2];
                int a = get_middle_point(i0, i1, middlePointIndexCache, temp_vertices_glm, radius); int b = get_middle_point(i1, i2, middlePointIndexCache, temp_vertices_glm, radius); int c = get_middle_point(i2, i0, middlePointIndexCache, temp_vertices_glm, radius);
                new_indices.insert(new_indices.end(), { (unsigned int)i0, (unsigned int)a, (unsigned int)c }); new_indices.insert(new_indices.end(), { (unsigned int)i1, (unsigned int)b, (unsigned int)a }); new_indices.insert(new_indices.end(), { (unsigned int)i2, (unsigned int)c, (unsigned int)b }); new_indices.insert(new_indices.end(), { (unsigned int)a,  (unsigned int)b, (unsigned int)c });
            }
            base_indices = new_indices;
        }
        mesh.indices = base_indices;
        for(const auto& v : temp_vertices_glm) {
            glm::vec3 norm = glm::normalize(v); mesh.vertices.push_back(v.x); mesh.vertices.push_back(v.y); mesh.vertices.push_back(v.z); mesh.normals.push_back(norm.x); mesh.normals.push_back(norm.y); mesh.normals.push_back(norm.z);
        }
        return mesh;
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

    // --- NEW: Wrappers for native Windows file dialogs ---
    std::string OpenFileDialog(SDL_Window* owner) {
        char filename[MAX_PATH] = { 0 };
        OPENFILENAMEA ofn = { 0 };
        ofn.lStructSize = sizeof(ofn);
        
        SDL_SysWMinfo wmInfo;
        SDL_VERSION(&wmInfo.version);
        SDL_GetWindowWMInfo(owner, &wmInfo);
        ofn.hwndOwner = wmInfo.info.win.window;

        ofn.lpstrFile = filename;
        ofn.nMaxFile = sizeof(filename);
        ofn.lpstrFilter = "Urbaxio Project (*.urbx)\0*.urbx\0All Files (*.*)\0*.*\0";
        ofn.nFilterIndex = 1;
        ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;

        if (GetOpenFileNameA(&ofn)) {
            return std::string(filename);
        }
        return "";
    }

    std::string SaveFileDialog(SDL_Window* owner) {
        char filename[MAX_PATH] = { 0 };
        OPENFILENAMEA ofn = { 0 };
        ofn.lStructSize = sizeof(ofn);

        SDL_SysWMinfo wmInfo;
        SDL_VERSION(&wmInfo.version);
        SDL_GetWindowWMInfo(owner, &wmInfo);
        ofn.hwndOwner = wmInfo.info.win.window;
        
        ofn.lpstrFile = filename;
        ofn.nMaxFile = sizeof(filename);
        ofn.lpstrFilter = "Urbaxio Project (*.urbx)\0*.urbx\0All Files (*.*)\0*.*\0";
        ofn.lpstrDefExt = "urbx";
        ofn.nFilterIndex = 1;
        ofn.Flags = OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR;

        if (GetSaveFileNameA(&ofn)) {
            return std::string(filename);
        }
        return "";
    }

    std::string OpenObjDialog(SDL_Window* owner) {
        char filename[MAX_PATH] = { 0 };
        OPENFILENAMEA ofn = { 0 };
        ofn.lStructSize = sizeof(ofn);
        SDL_SysWMinfo wmInfo;
        SDL_VERSION(&wmInfo.version);
        SDL_GetWindowWMInfo(owner, &wmInfo);
        ofn.hwndOwner = wmInfo.info.win.window;
        ofn.lpstrFile = filename;
        ofn.nMaxFile = sizeof(filename);
        ofn.lpstrFilter = "Wavefront OBJ (*.obj)\0*.obj\0All Files (*.*)\0*.*\0";
        ofn.nFilterIndex = 1;
        ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;
        if (GetOpenFileNameA(&ofn)) return std::string(filename);
        return "";
    }

    std::string SaveObjDialog(SDL_Window* owner) {
        char filename[MAX_PATH] = { 0 };
        OPENFILENAMEA ofn = { 0 };
        ofn.lStructSize = sizeof(ofn);
        SDL_SysWMinfo wmInfo;
        SDL_VERSION(&wmInfo.version);
        SDL_GetWindowWMInfo(owner, &wmInfo);
        ofn.hwndOwner = wmInfo.info.win.window;
        ofn.lpstrFile = filename;
        ofn.nMaxFile = sizeof(filename);
        ofn.lpstrFilter = "Wavefront OBJ (*.obj)\0*.obj\0All Files (*.*)\0*.*\0";
        ofn.lpstrDefExt = "obj";
        ofn.nFilterIndex = 1;
        ofn.Flags = OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR;
        if (GetSaveFileNameA(&ofn)) return std::string(filename);
        return "";
    }

    void FreeGPUResources(Urbaxio::Engine::SceneObject& obj) {
        if (obj.vao != 0) { glDeleteVertexArrays(1, &obj.vao); obj.vao = 0; }
        if (obj.vbo_vertices != 0) { glDeleteBuffers(1, &obj.vbo_vertices); obj.vbo_vertices = 0; }
        if (obj.vbo_normals != 0) { glDeleteBuffers(1, &obj.vbo_normals); obj.vbo_normals = 0; }
        if (obj.ebo != 0) { glDeleteBuffers(1, &obj.ebo); obj.ebo = 0; }
    }

    // --- NEW: Function to render the OBJ import options popup ---
    void RenderImportOptionsPopup(
        bool& show, 
        const std::string& filepath, 
        Urbaxio::Engine::Scene& scene) 
    {
        if (show) {
            ImGui::OpenPopup("Import Options");
            show = false; // Reset the trigger
        }

        ImGui::SetNextWindowSize(ImVec2(350, 180), ImGuiCond_FirstUseEver);
        if (ImGui::BeginPopupModal("Import Options", NULL, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::Text("Importing: %s", std::filesystem::path(filepath).filename().string().c_str());
            ImGui::Separator();

            static int selected_unit = 2; // Default to Millimeters
            static float custom_scale = 0.001f;

            ImGui::RadioButton("Meters (1.0)", &selected_unit, 0);
            ImGui::SameLine();
            ImGui::RadioButton("Centimeters (0.01)", &selected_unit, 1);
            ImGui::SameLine();
            ImGui::RadioButton("Millimeters (0.001)", &selected_unit, 2);
            
            if (selected_unit == 0) custom_scale = 1.0f;
            else if (selected_unit == 1) custom_scale = 0.01f;
            else if (selected_unit == 2) custom_scale = 0.001f;

            ImGui::Separator();

            if (ImGui::Button("Import", ImVec2(120, 0))) {
                Urbaxio::FileIO::ImportObjToScene(filepath, scene, custom_scale);
                ImGui::CloseCurrentPopup();
            }
            ImGui::SetItemDefaultFocus();
            ImGui::SameLine();
            if (ImGui::Button("Cancel", ImVec2(120, 0))) {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }
    }

    // --- NEW: Project Save/Load wrappers that also handle camera ---
    bool SaveProject(const std::string& path, Urbaxio::Engine::Scene* scene, const Urbaxio::Camera& camera) {
        std::ofstream file(path, std::ios::binary);
        if (!file.is_open()) return false;

        // 1. Header
        const char* magic = "URBX";
        uint32_t version = 1;
        file.write(magic, 4);
        file.write(reinterpret_cast<const char*>(&version), sizeof(version));

        // 2. Camera State
        file.write(reinterpret_cast<const char*>(&camera.Position), sizeof(camera.Position));
        file.write(reinterpret_cast<const char*>(&camera.Target), sizeof(camera.Target));

        // 3. Scene Data
        return scene->SaveToStream(file);
    }

    bool LoadProject(const std::string& path, Urbaxio::Engine::Scene* scene, Urbaxio::Camera& camera) {
        std::ifstream file(path, std::ios::binary);
        if (!file.is_open()) return false;

        // 1. Header
        char magic[4];
        uint32_t version;
        file.read(magic, 4);
        file.read(reinterpret_cast<char*>(&version), sizeof(version));
        if (std::string(magic, 4) != "URBX" || version != 1) {
            return false; // Invalid file
        }

        // 2. Camera State
        file.read(reinterpret_cast<char*>(&camera.Position), sizeof(camera.Position));
        file.read(reinterpret_cast<char*>(&camera.Target), sizeof(camera.Target));
        // Crucial step: Recalculate internal camera state (vectors, angles) from loaded pos/target
        camera.UpdateFromPositionTarget();
        
        // 3. Scene Data
        return scene->LoadFromStream(file);
    }

    void RecreateEssentialMarkers(Urbaxio::Engine::Scene* scene, float capsuleRadius, float capsuleHeight10m, float capsuleHeight5m) {
        if (!scene) return;
        // Center Marker
        Urbaxio::Engine::SceneObject* center_sphere = scene->create_object("CenterMarker");
        if (center_sphere) {
            center_sphere->set_mesh_buffers(CreateIcoSphereMesh(0.25f, 2));
            center_sphere->setExportable(false); // Do not export this
        }
        // Capsule Markers
        Urbaxio::Engine::SceneObject* cap10 = scene->create_object("UnitCapsuleMarker10m");
        if (cap10) {
            cap10->set_mesh_buffers(CreateCapsuleMesh(capsuleRadius, capsuleHeight10m));
            cap10->setExportable(false); // Do not export this
        }
        Urbaxio::Engine::SceneObject* cap5 = scene->create_object("UnitCapsuleMarker5m");
        if (cap5) {
            cap5->set_mesh_buffers(CreateCapsuleMesh(capsuleRadius, capsuleHeight5m));
            cap5->setExportable(false); // Do not export this
        }
    }

    // --- NEW: Factory function to create our VR panels ---
    void SetupVRPanels(Urbaxio::UI::VRUIManager& vruiManager, std::string& numpadInputTarget, Urbaxio::Tools::ToolManager& toolManager, bool& isNumpadActiveFlag, const Urbaxio::Tools::ToolContext& toolContext, unsigned int dragIconTexture) {
        {
            float panelScale = 0.392f;
            // Standard numpad position values
            glm::vec3 translation = glm::vec3(-0.009f, -0.059f, -0.148f);
            glm::vec3 eulerAnglesRad = glm::radians(glm::vec3(436.000f, 176.000f, 180.500f));
            glm::mat4 offset = glm::translate(glm::mat4(1.0f), translation) *
                               glm::mat4_cast(glm::quat(eulerAnglesRad)) *
                               glm::scale(glm::mat4(1.0f), glm::vec3(panelScale));
            
            auto& numpad = vruiManager.AddPanel("NewNumpad", glm::vec2(0.2f, 0.4f), offset);

            // Add Display Widget
            glm::vec2 displaySize(0.2f, 0.05f); 
            glm::vec3 displayCenter(0, 0.06f, 0.005f);
            numpad.AddWidget(std::make_unique<Urbaxio::UI::VRDisplayWidget>(displayCenter, displaySize, numpadInputTarget));

            // Top button row: 4 buttons (grab handle is at -1.5, then -0.5, 0.5, 1.5)
            float topButtonY = displayCenter.y + (displaySize.y * 0.5f + 0.012f + 0.01f);
            float topButtonSize = 0.025f;
            float topButtonSpacing = 0.04f;
            glm::vec3 whiteColor(1.0f);
            glm::vec3 greenColor(0.1f, 0.8f, 0.2f);

            // Placeholder button 1 (grab handle is now an icon button)
            numpad.AddWidget(std::make_unique<Urbaxio::UI::VRConfirmButtonWidget>(
                glm::vec3(-1.5f * topButtonSpacing, topButtonY, 0.01f), topButtonSize, dragIconTexture, [] {}
            ));

            // Placeholder button 2 (second position)
            numpad.AddWidget(std::make_unique<Urbaxio::UI::VRConfirmButtonWidget>(
                glm::vec3(-0.5f * topButtonSpacing, topButtonY, 0.01f), topButtonSize, whiteColor, []{}
            ));

            // Confirm button at the top (fourth position)
            auto confirmCallback = [&toolManager, &isNumpadActiveFlag, &numpadInputTarget, &toolContext]() {
                float length_mm;
                auto result = std::from_chars(numpadInputTarget.data(), numpadInputTarget.data() + numpadInputTarget.size(), length_mm);
                if (result.ec != std::errc()) {
                    numpadInputTarget = "0";
                    return;
                }

                float length_m = length_mm / 1000.0f;
                if (length_m < 1e-4f) {
                    isNumpadActiveFlag = false;
                    numpadInputTarget = "0";
                    return;
                }

                if (toolManager.GetActiveToolType() == Urbaxio::Tools::ToolType::Line) {
                    static_cast<Urbaxio::Tools::LineTool*>(toolManager.GetActiveTool())->SetLengthInput(numpadInputTarget);
                    toolManager.GetActiveTool()->OnKeyDown(SDLK_RETURN, *toolContext.shiftDown, *toolContext.ctrlDown);
                } else if (toolManager.GetActiveToolType() == Urbaxio::Tools::ToolType::PushPull) {
                    static_cast<Urbaxio::Tools::PushPullTool*>(toolManager.GetActiveTool())->SetLengthInput(numpadInputTarget);
                    toolManager.GetActiveTool()->OnKeyDown(SDLK_RETURN, *toolContext.shiftDown, *toolContext.ctrlDown);
                }

                numpadInputTarget = "0";
                isNumpadActiveFlag = false;
            };
            numpad.AddWidget(std::make_unique<Urbaxio::UI::VRConfirmButtonWidget>(
                glm::vec3(1.5f * topButtonSpacing, topButtonY, 0.01f), topButtonSize, greenColor, confirmCallback
            ));

            // Add Buttons (copied from old code)
            float keySpacing = 0.06f;
            float keyTextHeight = 0.03f;
            
            const char* keys[] = {"1","2","3", "4","5","6", "7","8","9", ".","0","<-"};
            for (int i = 0; i < 12; ++i) {
                int row = i / 3;
                int col = i % 3;
                glm::vec3 keyCenter(0,0,0);

                if (i < 9) { // 1-9
                     keyCenter = glm::vec3(((float)col - 1.0f) * keySpacing, 0.0f - (float)row * keySpacing, 0.01f);
                } else if (i == 9) { // '.'
                     keyCenter = glm::vec3(-1.0f * keySpacing, -3.0f * keySpacing, 0.01f);
                } else if (i == 10) { // '0'
                     keyCenter = glm::vec3(0.0f, -3.0f * keySpacing, 0.01f);
                } else if (i == 11) { // Backspace
                     keyCenter = glm::vec3(1.0f * keySpacing, -3.0f * keySpacing, 0.01f);
                }

                std::string keyStr = keys[i];
                auto callback = [keyStr, &numpadInputTarget]() {
                    if (keyStr == "<-") {
                        if (numpadInputTarget.length() > 1) numpadInputTarget.pop_back(); else numpadInputTarget = "0";
                    } else if (keyStr == ".") {
                        if (numpadInputTarget.find('.') == std::string::npos) numpadInputTarget += ".";
                    } else {
                        // Logic from old numpad
                        if (numpadInputTarget == "0" && keyStr != "0") numpadInputTarget.clear();
                        if (numpadInputTarget != "0" || keyStr != "0") numpadInputTarget += keyStr;
                    }
                };
                numpad.AddWidget(std::make_unique<Urbaxio::UI::VRButtonWidget>(keyStr, keyCenter, glm::vec2(keyTextHeight), callback));
            }
        }
    }

    // --- NEW: Factory function to create the tool menu panel ---
    void SetupToolMenuPanel(Urbaxio::UI::VRUIManager& vruiManager, Urbaxio::Tools::ToolManager& toolManager, unsigned int dragIconTexture) {
        // 1. Define panel properties
        // Offset from the left controller's grip pose
        // Positioned to the right of the controller and tilted for easy viewing
        glm::mat4 panelOffset = 
            glm::translate(glm::mat4(1.0f), glm::vec3(0.1f, 0.0f, 0.0f)) * 
            glm::rotate(glm::mat4(1.0f), glm::radians(-70.0f), glm::vec3(1.0f, 0.0f, 0.0f));

        float buttonHeight = 0.025f; // This now defines the sphere diameter
        float buttonSpacing = 0.04f;
        float panelWidth = buttonHeight * 1.2f; // Panel is just a bit wider than the spheres
        float panelHeight = (5 * buttonSpacing); // Height for 4 buttons + 1 grab handle

        auto& toolMenu = vruiManager.AddPanel("ToolMenu", glm::vec2(panelWidth, panelHeight), panelOffset);

        // 1. Add Grab Handle widget at the top
        glm::vec3 grabHandlePos(0.0f, (panelHeight / 2.0f) - (buttonSpacing * 0.5f), 0.01f);
        toolMenu.AddWidget(std::make_unique<Urbaxio::UI::VRConfirmButtonWidget>(
            grabHandlePos, buttonHeight, dragIconTexture, [] {}
        ));

        // 2. Add widgets (buttons) for each tool
        glm::vec3 textCenterPos = grabHandlePos - glm::vec3(0, buttonSpacing, 0);
        toolMenu.AddWidget(std::make_unique<Urbaxio::UI::VRToolButtonWidget>("Select", textCenterPos, glm::vec2(panelWidth, buttonHeight), 
            Urbaxio::Tools::ToolType::Select, toolManager,
            [&toolManager]() { toolManager.SetTool(Urbaxio::Tools::ToolType::Select); }
        ));

        glm::vec3 linePos = textCenterPos - glm::vec3(0, buttonSpacing, 0);
        toolMenu.AddWidget(std::make_unique<Urbaxio::UI::VRToolButtonWidget>("Line", linePos, glm::vec2(panelWidth, buttonHeight),
            Urbaxio::Tools::ToolType::Line, toolManager,
            [&toolManager]() { toolManager.SetTool(Urbaxio::Tools::ToolType::Line); }
        ));

        glm::vec3 pushPullPos = linePos - glm::vec3(0, buttonSpacing, 0);
        toolMenu.AddWidget(std::make_unique<Urbaxio::UI::VRToolButtonWidget>("Push/Pull", pushPullPos, glm::vec2(panelWidth, buttonHeight),
            Urbaxio::Tools::ToolType::PushPull, toolManager,
            [&toolManager]() { toolManager.SetTool(Urbaxio::Tools::ToolType::PushPull); }
        ));

        glm::vec3 movePos = pushPullPos - glm::vec3(0, buttonSpacing, 0);
        toolMenu.AddWidget(std::make_unique<Urbaxio::UI::VRToolButtonWidget>("Move", movePos, glm::vec2(panelWidth, buttonHeight),
            Urbaxio::Tools::ToolType::Move, toolManager,
            [&toolManager]() { toolManager.SetTool(Urbaxio::Tools::ToolType::Move); }
        ));
    }

    unsigned int LoadTextureFromFile(const std::string& path) {
        int width, height, channels;
        stbi_set_flip_vertically_on_load(true);
        unsigned char* data = stbi_load(path.c_str(), &width, &height, &channels, 4);
        if (data == nullptr) {
            std::cerr << "Failed to load texture: " << path << std::endl;
            return 0;
        }

        unsigned int textureID;
        glGenTextures(1, &textureID);
        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        stbi_image_free(data);
        return textureID;
    }

} // end anonymous namespace

// --- GPU Mesh Upload Helper ---
bool UploadMeshToGPU(Urbaxio::Engine::SceneObject& object) { /* ... */ const Urbaxio::CadKernel::MeshBuffers& mesh = object.get_mesh_buffers(); if (mesh.isEmpty() || mesh.normals.empty()) { if (mesh.normals.empty() && !mesh.vertices.empty()) { std::cerr << "UploadMeshToGPU: Mesh for object " << object.get_id() << " is missing normals!" << std::endl; } return false; } if (object.vao != 0) glDeleteVertexArrays(1, &object.vao); if (object.vbo_vertices != 0) glDeleteBuffers(1, &object.vbo_vertices); if (object.vbo_normals != 0) glDeleteBuffers(1, &object.vbo_normals); if (object.ebo != 0) glDeleteBuffers(1, &object.ebo); object.vao = object.vbo_vertices = object.vbo_normals = object.ebo = 0; object.index_count = 0; glGenVertexArrays(1, &object.vao); if (object.vao == 0) return false; glBindVertexArray(object.vao); glGenBuffers(1, &object.vbo_vertices); if (object.vbo_vertices == 0) { glDeleteVertexArrays(1, &object.vao); object.vao = 0; return false; } glBindBuffer(GL_ARRAY_BUFFER, object.vbo_vertices); glBufferData(GL_ARRAY_BUFFER, mesh.vertices.size() * sizeof(float), mesh.vertices.data(), GL_STATIC_DRAW); glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0); glEnableVertexAttribArray(0); glGenBuffers(1, &object.vbo_normals); if (object.vbo_normals == 0) { glDeleteBuffers(1, &object.vbo_vertices); glDeleteVertexArrays(1, &object.vao); object.vao = object.vbo_vertices = 0; return false; } glBindBuffer(GL_ARRAY_BUFFER, object.vbo_normals); glBufferData(GL_ARRAY_BUFFER, mesh.normals.size() * sizeof(float), mesh.normals.data(), GL_STATIC_DRAW); glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0); glEnableVertexAttribArray(1); glGenBuffers(1, &object.ebo); if (object.ebo == 0) { glDeleteBuffers(1, &object.vbo_normals); glDeleteBuffers(1, &object.vbo_vertices); glDeleteVertexArrays(1, &object.vao); object.vao = object.vbo_vertices = object.vbo_normals = 0; return false; } glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, object.ebo); glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh.indices.size() * sizeof(unsigned int), mesh.indices.data(), GL_STATIC_DRAW); object.index_count = static_cast<GLsizei>(mesh.indices.size()); glBindVertexArray(0); glBindBuffer(GL_ARRAY_BUFFER, 0); glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0); std::cout << "UploadMeshToGPU: Successfully uploaded mesh for object " << object.get_id() << std::endl; return true; }

int main(int argc, char* argv[]) {
    std::cout << "Shell: Starting Urbaxio Application..." << std::endl;
    // --- Initialization ---
    initialize_engine(); Urbaxio::Engine::Scene* scene_ptr = reinterpret_cast<Urbaxio::Engine::Scene*>(get_engine_scene()); if (!scene_ptr) return 1; if (SDL_Init(SDL_INIT_VIDEO) != 0) return 1; const char* glsl_version = "#version 430 core"; SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_DEBUG_FLAG); SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4); SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3); SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE); SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1); SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24); SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8); SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI | SDL_WINDOW_SHOWN); SDL_Window* window = SDL_CreateWindow("Urbaxio", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720, window_flags); if (!window) { SDL_Quit(); return 1; } SDL_GLContext gl_context = SDL_GL_CreateContext(window); if (!gl_context) { SDL_DestroyWindow(window); SDL_Quit(); return 1; } SDL_GL_MakeCurrent(window, gl_context); SDL_GL_SetSwapInterval(1); if (!gladLoadGLLoader((GLADloadproc)SDL_GL_GetProcAddress)) { return 1; } std::cout << "Shell: OpenGL Initialized: V:" << glGetString(GL_VERSION) << std::endl; std::cout << "Shell: OpenGL Renderer: " << glGetString(GL_RENDERER) << std::endl; std::cout << "Shell: OpenGL Vendor: " << glGetString(GL_VENDOR) << std::endl; IMGUI_CHECKVERSION(); ImGui::CreateContext(); ImGuiIO& io = ImGui::GetIO(); io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; ImGui::StyleColorsDark(); if (!ImGui_ImplSDL2_InitForOpenGL(window, gl_context)) return 1; if (!ImGui_ImplOpenGL3_Init(glsl_version)) return 1;     std::cout << "Shell: All subsystems initialized." << std::endl;

    // --- NEW: Enable Drag and Drop events for the window ---
    SDL_EventState(SDL_DROPFILE, SDL_ENABLE);

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

    // State for modifier keys and VR input mode, managed by the main loop
    bool shiftDown = false;
    bool ctrlDown = false;
    bool numpadInputActive = false;

    Urbaxio::Tools::ToolContext toolContext;
    toolContext.scene = scene_ptr;
    toolContext.camera = &camera;
    toolContext.window = window;
    toolContext.display_w = &display_w;
    toolContext.display_h = &display_h;
    toolContext.shiftDown = &shiftDown;
    toolContext.ctrlDown = &ctrlDown;
    toolContext.isNumpadActive = &numpadInputActive;
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

    // --- Create all essential markers ---
    RecreateEssentialMarkers(scene_ptr, capsuleRadius, capsuleHeight10m, capsuleHeight5m);

    // --- NEW: Create VR Controller Visuals ---
    Urbaxio::Engine::SceneObject* leftControllerVisual = nullptr;
    Urbaxio::Engine::SceneObject* rightControllerVisual = nullptr;
    if (vr_mode) {
        // --- NEW: Load controller model from OBJ instead of creating cubes ---
        std::string controllerObjPath = "../../resources/controller.obj";
        if (!std::filesystem::exists(controllerObjPath)) {
            controllerObjPath = "../../../resources/controller.obj";
        }
        Urbaxio::CadKernel::MeshBuffers controllerMesh = Urbaxio::FileIO::LoadMeshFromObj(controllerObjPath);

        if (!controllerMesh.isEmpty()) {
            leftControllerVisual = scene_ptr->create_object("LeftControllerVisual");
            if (leftControllerVisual) {
                leftControllerVisual->set_mesh_buffers(controllerMesh); // Makes a copy
                leftControllerVisual->setExportable(false);
            }
            rightControllerVisual = scene_ptr->create_object("RightControllerVisual");
            if (rightControllerVisual) {
                rightControllerVisual->set_mesh_buffers(controllerMesh); // Makes another copy
                rightControllerVisual->setExportable(false);
            }
            std::cout << "Shell: Successfully loaded controller model." << std::endl;
        } else {
            std::cerr << "Shell Warning: Could not load controller model from " << controllerObjPath 
                      << ". Falling back to cubes." << std::endl;
        leftControllerVisual = scene_ptr->create_box_object("LeftControllerVisual", 1.0, 1.0, 1.0);
        if(leftControllerVisual) leftControllerVisual->setExportable(false);
        rightControllerVisual = scene_ptr->create_box_object("RightControllerVisual", 1.0, 1.0, 1.0);
        if(rightControllerVisual) rightControllerVisual->setExportable(false);
        }
    }
    // --- NEW: Live tuning variables for controller model offset, initialized with tuned values ---
    static glm::vec3 g_controllerOffsetTranslate(0.012f, 0.002f, 0.006f);
    static glm::vec3 g_controllerOffsetEuler(273.000f, 166.500f, 81.000f);
    static float g_controllerModelScale = 0.7814f;

    // --- NEW: VR UI Manager ---
    Urbaxio::UI::VRUIManager vruiManager;
    std::string g_newNumpadInput = "0";

    // --- NEW: Load UI Icons ---
    unsigned int dragIconTexture = 0;
    std::string dragIconPath = "../../resources/drag_icon.png";
    if (!std::filesystem::exists(dragIconPath)) {
        dragIconPath = "../../../resources/drag_icon.png";
    }
    dragIconTexture = LoadTextureFromFile(dragIconPath);

    // --- NEW: Setup our VR panels using the new system ---
    SetupVRPanels(vruiManager, g_newNumpadInput, toolManager, numpadInputActive, toolContext, dragIconTexture);
    SetupToolMenuPanel(vruiManager, toolManager, dragIconTexture);

    // --- NEW: State for the import options dialog ---
    bool g_showImportOptionsPopup = false;
    std::string g_fileToImportPath;

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

        // --- NEW: Handle file drop after processing events ---
        if (!inputHandler.droppedFilePath.empty()) {
            std::filesystem::path droppedPath(inputHandler.droppedFilePath);
            if (droppedPath.extension() == ".urbx") {
                // It's a project file, load it directly
                for (auto* obj : scene_ptr->get_all_objects()) { if (obj) FreeGPUResources(*obj); }
                LoadProject(inputHandler.droppedFilePath, scene_ptr, camera);
                RecreateEssentialMarkers(scene_ptr, capsuleRadius, capsuleHeight10m, capsuleHeight5m);
            } else if (droppedPath.extension() == ".obj") {
                // It's an OBJ, trigger the import options popup
                g_fileToImportPath = inputHandler.droppedFilePath;
                g_showImportOptionsPopup = true;
            }
            
            inputHandler.droppedFilePath.clear(); // Clear the path to prevent re-triggering
        }

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
            ImGui::Text("File:");
            if (ImGui::Button("New Scene")) {
                for (auto* obj : scene_ptr->get_all_objects()) {
                    if (obj) FreeGPUResources(*obj);
                }
                scene_ptr->NewScene();
                RecreateEssentialMarkers(scene_ptr, capsuleRadius, capsuleHeight10m, capsuleHeight5m);
            }
            ImGui::SameLine();
            if (ImGui::Button("Save Scene")) {
                std::string path = SaveFileDialog(window);
                if (!path.empty()) {
                    SaveProject(path, scene_ptr, camera);
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Open Scene")) {
                std::string path = OpenFileDialog(window);
                if (!path.empty()) {
                    for (auto* obj : scene_ptr->get_all_objects()) { if (obj) FreeGPUResources(*obj); }
                    LoadProject(path, scene_ptr, camera);
                    RecreateEssentialMarkers(scene_ptr, capsuleRadius, capsuleHeight10m, capsuleHeight5m);
                }
            }

            ImGui::SeparatorText("Exchange");
            if (ImGui::Button("Import OBJ")) {
                std::string path = OpenObjDialog(window);
                if (!path.empty()) {
                    g_fileToImportPath = path;
                    g_showImportOptionsPopup = true;
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Export OBJ")) {
                std::string path = SaveObjDialog(window);
                if (!path.empty()) {
                    Urbaxio::FileIO::ExportSceneToObj(path, *scene_ptr);
                }
            }

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
                    for (auto* obj : objects_to_clean) { if (obj) FreeGPUResources(*obj); }
                    scene_ptr->TestFaceSplitting(); 
                }
            }
            
            // --- Tool-specific UI ---
            toolManager.RenderUI();

            // --- RE-ADDED: Controller model tuning UI ---
            if (vr_mode && ImGui::CollapsingHeader("Controller Model Debug")) {
                ImGui::Text("Tune the RIGHT controller. The left will mirror automatically.");
                ImGui::DragFloat3("Offset Translate", &g_controllerOffsetTranslate.x, 0.001f);
                ImGui::DragFloat3("Offset Rotate (Euler)", &g_controllerOffsetEuler.x, 0.5f);
                ImGui::DragFloat("Model Scale", &g_controllerModelScale, 0.0001f, 0.0f, 2.0f, "%.4f");
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
                    // This is tricky as we don't store the IDs. Easiest is to find by name.
                    for (auto* obj : scene_ptr->get_all_objects()) {
                        if (obj && obj->get_name() == "UnitCapsuleMarker10m") {
                            obj->set_mesh_buffers(CreateCapsuleMesh(capsuleRadius, capsuleHeight10m));
                            obj->vao = 0;
                            break;
                        }
                    }
                }
                if (radius_changed || height5m_changed) {
                    for (auto* obj : scene_ptr->get_all_objects()) {
                         if (obj && obj->get_name() == "UnitCapsuleMarker5m") {
                            obj->set_mesh_buffers(CreateCapsuleMesh(capsuleRadius, capsuleHeight5m));
                            obj->vao = 0;
                            break;
                        }
                    }
                }
            }
            ImGui::End();
        }

        // --- NEW: Render our custom modal popup if it's been triggered ---
        RenderImportOptionsPopup(g_showImportOptionsPopup, g_fileToImportPath, *scene_ptr);

        ImGui::Render();

        if (vr_mode && vrManager->IsInitialized()) {
            // --- VR RENDER PATH ---
            uint32_t leftEyeImageIndex = -1; // For mirror view

            if (vrManager->BeginFrame()) {
                // Poll controller state after syncing actions in BeginFrame
                vrManager->PollActions();
                const auto& leftHand = vrManager->GetLeftHandVisual();
                // Update modifier state for VR after polling actions

                // --- NEW: Handle Undo/Redo gesture actions ---
                if (vrManager->triggeredUndoRedoAction == Urbaxio::UndoRedoAction::TriggerUndo) {
                    scene_ptr->getCommandManager()->Undo();
                    vrManager->triggeredUndoRedoAction = Urbaxio::UndoRedoAction::None; // Reset the flag
                } else if (vrManager->triggeredUndoRedoAction == Urbaxio::UndoRedoAction::TriggerRedo) {
                    scene_ptr->getCommandManager()->Redo();
                    vrManager->triggeredUndoRedoAction = Urbaxio::UndoRedoAction::None; // Reset the flag
                }


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

                // --- NEW: Calculate UNSCALED transforms for parenting UI elements ---
                glm::mat4 leftControllerUnscaledTransform(1.0f);
                glm::mat4 rightControllerUnscaledTransform(1.0f);

                if (leftHand.isValid && leftControllerVisual) {
                    glm::mat4 rawPoseMatrix = XrPoseToModelMatrix(leftHand.pose);
                    leftControllerUnscaledTransform = worldTransform * rawPoseMatrix;

                    // Build the mirrored local transform for the left hand
                    glm::mat4 localOffset =
                        glm::translate(glm::mat4(1.0f), g_controllerOffsetTranslate * glm::vec3(-1, 1, 1)) *
                        glm::mat4_cast(glm::quat(glm::radians(g_controllerOffsetEuler * glm::vec3(1, -1, -1))));

                    glm::mat4 scaleMatrix = glm::scale(glm::mat4(1.0f), glm::vec3(-g_controllerModelScale, g_controllerModelScale, g_controllerModelScale));
                    transformOverrides[leftControllerVisual->get_id()] = leftControllerUnscaledTransform * localOffset * scaleMatrix;

                    colorOverrides[leftControllerVisual->get_id()] = MixColorFromPress(leftHand.pressValue);
                    unlitOverrides[leftControllerVisual->get_id()] = true;
                }

                if (rightHand.isValid && rightControllerVisual) {
                    glm::mat4 rawPoseMatrix = XrPoseToModelMatrix(rightHand.pose);
                    rightControllerUnscaledTransform = worldTransform * rawPoseMatrix;

                    // Build the standard local transform for the right hand from the UI values
                    glm::mat4 localOffset =
                        glm::translate(glm::mat4(1.0f), g_controllerOffsetTranslate) *
                        glm::mat4_cast(glm::quat(glm::radians(g_controllerOffsetEuler)));

                    glm::mat4 scaleMatrix = glm::scale(glm::mat4(1.0f), glm::vec3(g_controllerModelScale));
                    transformOverrides[rightControllerVisual->get_id()] = rightControllerUnscaledTransform * localOffset * scaleMatrix;

                    colorOverrides[rightControllerVisual->get_id()] = MixColorFromPress(rightHand.pressValue);
                    unlitOverrides[rightControllerVisual->get_id()] = true;
                }

                // Update VR pointer with menu clipping
                if (rightHand.isValid) {
                    float rayLength = 100.0f;
                    glm::vec3 rayEnd = (vrSnap.snapped && vrSnap.type != Urbaxio::SnapType::GRID) 
                                       ? vrSnap.worldPoint 
                                       : vrRayOrigin + vrRayDirection * rayLength;

                    // NEW: Check intersection with ToolMenu panel
                    if (auto* toolMenuPanel = vruiManager.GetPanel("ToolMenu")) {
                        if (toolMenuPanel->alpha > 0.01f) {
                            Urbaxio::UI::Ray testRay = {vrRayOrigin, vrRayDirection};
                            Urbaxio::UI::HitResult panelHit = toolMenuPanel->CheckIntersection(testRay, leftControllerUnscaledTransform);
                            if (panelHit.didHit && panelHit.distance < glm::distance(vrRayOrigin, rayEnd)) {
                                rayEnd = vrRayOrigin + vrRayDirection * panelHit.distance;
                            }
                        }
                    }
                    
                    // Check if pointer intersects with new numpad panel and shorten the ray
                    if (auto* numpadPanel = vruiManager.GetPanel("NewNumpad")) {
                        if (numpadPanel->alpha > 0.01f) {
                            Urbaxio::UI::Ray testRay = {vrRayOrigin, vrRayDirection};
                            Urbaxio::UI::HitResult panelHit = numpadPanel->CheckIntersection(testRay, leftControllerUnscaledTransform);
                            if (panelHit.didHit && panelHit.distance < glm::distance(vrRayOrigin, rayEnd)) {
                                rayEnd = vrRayOrigin + vrRayDirection * panelHit.distance;
                            }
                        }
                    }
                    
                    renderer.UpdateVRPointer(vrRayOrigin, rayEnd, true);
                }

                    // --- NEW: Update and Render VR UI Manager ---
                    Urbaxio::UI::Ray worldRay = {vrRayOrigin, vrRayDirection};
                    
                    vruiManager.Update(worldRay, leftControllerUnscaledTransform, rightControllerUnscaledTransform, rightHand.triggerClicked);

                    // --- NEW: Handle grabbing for the tool menu panel ---
                    if (auto* toolMenuPanel = vruiManager.GetPanel("ToolMenu")) {
                        // Check hover on the specific widget (grab handle is the first widget)
                        bool isHoveringGrabHandle = toolMenuPanel->IsVisible() && toolMenuPanel->alpha > 0.01f && 
                                                    vruiManager.GetHoveredPanel() == toolMenuPanel && 
                                                    toolMenuPanel->GetHoveredWidget() == toolMenuPanel->GetWidget(0);

                        if (rightHand.triggerClicked && isHoveringGrabHandle) {
                            toolMenuPanel->isGrabbing = true;
                            toolMenuPanel->grabbedInitialTransform = toolMenuPanel->transform;
                            toolMenuPanel->grabbedControllerInitialTransform = rightControllerUnscaledTransform;
                        }
                        if (rightHand.triggerReleased && toolMenuPanel->isGrabbing) {
                            toolMenuPanel->isGrabbing = false;
                            toolMenuPanel->GetOffsetTransform() = glm::inverse(leftControllerUnscaledTransform) * toolMenuPanel->transform;
                        }

                        if(toolMenuPanel->isGrabbing && rightHand.isValid) {
                            glm::mat4 deltaTransform = rightControllerUnscaledTransform * glm::inverse(toolMenuPanel->grabbedControllerInitialTransform);
                            toolMenuPanel->transform = deltaTransform * toolMenuPanel->grabbedInitialTransform;
                        }
                    }

                    // Handle grabbing for the new panel
                    if (auto* numpadPanel = vruiManager.GetPanel("NewNumpad")) {
                        // MODIFIED: Use the same robust hover check as the tool menu
                        // FIX: The grab handle on the numpad is the SECOND widget (index 1), not the first.
                        bool isHoveringNumpadGrabHandle = numpadPanel->IsVisible() && numpadPanel->alpha > 0.01f && 
                                                          vruiManager.GetHoveredPanel() == numpadPanel && 
                                                          numpadPanel->GetHoveredWidget() == numpadPanel->GetWidget(1);

                        if (rightHand.triggerClicked && isHoveringNumpadGrabHandle) {
                             numpadPanel->isGrabbing = true;
                             numpadPanel->grabbedInitialTransform = numpadPanel->transform;
                             numpadPanel->grabbedControllerInitialTransform = rightControllerUnscaledTransform;
                        }
                        if (rightHand.triggerReleased && numpadPanel->isGrabbing) {
                            numpadPanel->isGrabbing = false;
                            numpadPanel->GetOffsetTransform() = glm::inverse(leftControllerUnscaledTransform) * numpadPanel->transform;
                        }

                        if(numpadPanel->isGrabbing && rightHand.isValid) {
                             glm::mat4 deltaTransform = rightControllerUnscaledTransform * glm::inverse(numpadPanel->grabbedControllerInitialTransform);
                             numpadPanel->transform = deltaTransform * numpadPanel->grabbedInitialTransform;
                        }
                    }

                // --- VR Tool Interaction Logic ---
                if (toolManager.GetActiveTool()) {
                    // Only update hover if a VR UI Panel is not being interacted with
                    if (vruiManager.GetHoveredPanel() == nullptr) {
                    if (toolManager.GetActiveToolType() == Urbaxio::Tools::ToolType::PushPull) {
                        static_cast<Urbaxio::Tools::PushPullTool*>(toolManager.GetActiveTool())->updateHover(vrRayOrigin, vrRayDirection);
                    }
                    } else {
                        // Clear hover if pointer is on any panel
                        *toolContext.hoveredObjId = 0;
                        toolContext.hoveredFaceTriangleIndices->clear();
                    }
                    // Step 2: Update tool with current snap result for drawing/actions
                    toolManager.GetActiveTool()->OnUpdate(vrSnap, vrRayOrigin, vrRayDirection);
                }

                // --- Unified UI State Update (Menus, Numpad Visibility & Display Text) ---
                const float MENU_FADE_SPEED = 0.1f;
                bool isTriggerPressed = vrManager->rawLeftTriggerValue > 0.5f;

                // Update tool menu alpha
                if (auto* toolMenuPanel = vruiManager.GetPanel("ToolMenu")) {
                    if (isTriggerPressed) {
                        toolMenuPanel->alpha = std::min(1.0f, toolMenuPanel->alpha + MENU_FADE_SPEED);
                    } else {
                        toolMenuPanel->alpha = std::max(0.0f, toolMenuPanel->alpha - MENU_FADE_SPEED);
                    }
                    toolMenuPanel->SetVisible(toolMenuPanel->alpha > 0.01f);
                }

                // Update numpad alpha, visibility, and display text
                if (auto* numpadPanel = vruiManager.GetPanel("NewNumpad")) {
                    // The numpad is visible ONLY when the trigger is held. No other conditions.
                    if (isTriggerPressed) {
                        numpadPanel->alpha = std::min(1.0f, numpadPanel->alpha + MENU_FADE_SPEED);
                    } else {
                        numpadPanel->alpha = std::max(0.0f, numpadPanel->alpha - MENU_FADE_SPEED);
                    }
                    numpadPanel->SetVisible(numpadPanel->alpha > 0.01f);

                    // Only update the display automatically if the user is NOT typing
                    if (!numpadInputActive) {
                        bool displayUpdated = false;
                        if (toolManager.GetActiveToolType() == Urbaxio::Tools::ToolType::Line) {
                            auto* lineTool = static_cast<Urbaxio::Tools::LineTool*>(toolManager.GetActiveTool());
                            if (lineTool->IsDrawing()) {
                                g_newNumpadInput = fmt::format("{:.0f}", lineTool->GetCurrentLineLength() * 1000.0f);
                                displayUpdated = true;
                            }
                        } else if (toolManager.GetActiveToolType() == Urbaxio::Tools::ToolType::PushPull) {
                            auto* pushPullTool = static_cast<Urbaxio::Tools::PushPullTool*>(toolManager.GetActiveTool());
                            if (pushPullTool->IsPushPullActive()) {
                                g_newNumpadInput = fmt::format("{:.0f}", std::abs(pushPullTool->GetCurrentLength() * 1000.0f));
                                displayUpdated = true;
                            }
                        }

                        if (!displayUpdated) {
                            g_newNumpadInput = "0";
                        }
                    }
                }
                
                if (vrManager->leftAButtonDoubleClicked) {
                    if (toolManager.GetActiveToolType() == Urbaxio::Tools::ToolType::Line) {
                        auto* lineTool = static_cast<Urbaxio::Tools::LineTool*>(toolManager.GetActiveTool());
                        // Allow toggling input only if a line is being drawn, or if we are turning it off
                        if (lineTool->IsDrawing() || numpadInputActive) {
                            numpadInputActive = !numpadInputActive;
                            if (numpadInputActive) {
                                g_newNumpadInput = "0"; // Reset on activation
                            }
                        }
                    } else if (toolManager.GetActiveToolType() == Urbaxio::Tools::ToolType::PushPull) {
                        auto* pushPullTool = static_cast<Urbaxio::Tools::PushPullTool*>(toolManager.GetActiveTool());
                        // Allow toggling input only if a push/pull is in progress, or if we are turning it off
                        if (pushPullTool->IsPushPullActive() || numpadInputActive) {
                            numpadInputActive = !numpadInputActive;
                            if (numpadInputActive) {
                                g_newNumpadInput = "0"; // Reset on activation
                            }
                        }
                    }
                }
                
                if (rightHand.triggerClicked) {
                    bool clickConsumed = vruiManager.HandleClick();
                    
                    if (!clickConsumed) {
                            // If not interacting with UI, it's a world action
                            toolManager.OnLeftMouseDown(0, 0, *toolContext.shiftDown, *toolContext.ctrlDown, vrRayOrigin, vrRayDirection);
                    }
                }
                if (rightHand.triggerReleased) {
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
                    
                    // --- Hack to pass view matrix to UI widgets ---
                    renderer.vrMenuWidgetShaderProgram_viewMatrix_HACK = view;
                    // --- End Hack ---
                    
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
                    
                    // --- NEW: Render VR UI Manager ---
                    vruiManager.Render(renderer, textRenderer, view, projection);

                    
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
                    // MODIFIED: Render only the non-panel text (like distance text). Panel text is now rendered inside vruiManager.
                    textRenderer.Render(view, projection);

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
    if (scene_ptr) { for (auto* obj : scene_ptr->get_all_objects()) { if (obj) FreeGPUResources(*obj); } } ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplSDL2_Shutdown(); ImGui::DestroyContext(); SDL_GL_DeleteContext(gl_context); SDL_DestroyWindow(window); SDL_Quit(); std::cout << "Shell: Urbaxio Application finished gracefully." << std::endl; return 0;
}