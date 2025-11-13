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
#include <engine/MaterialManager.h>
#include <engine/geometry/MeshGeometry.h>
// --- ADD THESE INCLUDES ---
#include <engine/geometry/BRepGeometry.h>
#include <engine/commands/VoxelizeCommand.h>
// --- NEW: Include volumetric geometry headers ---
#include <engine/geometry/VoxelGrid.h>
#include <engine/geometry/VolumetricGeometry.h>
#include <cad_kernel/cad_kernel.h>
#include <tools/ToolManager.h>
#include <tools/SelectTool.h>
#include <tools/LineTool.h>
#include <tools/MoveTool.h>
#include <tools/PushPullTool.h>
// <-- NEW
#include <tools/PaintTool.h>
#include <tools/SculptTool.h>

#include "camera.h"
#include "input_handler.h"
#include "renderer.h"
#include "VRManager.h"

#include <SDL2/SDL.h>
#include <SDL2/SDL_syswm.h>
#include <glad/glad.h>
#include "stb_image.h"

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
#include <functional>
#include <set>
#include <map>
#include <cstdint>
#include <thread>
#include <atomic>

#include "TextRenderer.h"
#include "file_io.h"
#include "snapping.h"
#include <filesystem>
#include "LoadingManager.h"
// --- NEW: VR UI System includes ---
#include "ui/IVRWidget.h"
#include "ui/VRButtonWidget.h"
#include "ui/VRConfirmButtonWidget.h"
#include "ui/VRMenuSphereWidget.h"
#include "ui/VRToolButtonWidget.h"
#include "ui/VRDisplayWidget.h"
#include "ui/VRUIManager.h"
#include "ui/VRPanel.h"
#include "ui/Layouts.h"
// ДОБАВЬТЕ ЭТУ СТРОКУ:
#include "ui/VRScrollWidget.h"
// --- VR menu interaction helpers ---
#include <glm/gtx/intersect.hpp>

#include <fstream>
#include <charconv>
// --- ДОБАВЬ ЭТУ СТРОКУ ---
#include <SDL2/SDL_keycode.h>
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
    // --- START OF MODIFICATION: Replace the entire VRPanelRowWidget class with this ---
    class VRPanelRowWidget : public Urbaxio::UI::IVRWidget {
    public:
        VRPanelRowWidget(const std::string& panelName, Urbaxio::UI::VRUIManager& manager, const glm::vec2& size,
                         unsigned int backIcon, unsigned int minIcon, unsigned int closeIcon) 
            : panelName_(panelName), manager_(manager), size_(size), localPosition_(0.0f), hoverAnimationT_(0.0f),
              isMinimized_(false), isVisible_(true)
        {
            float sphereDiameter = size.y * 0.7f;
            float sphereSpacing = sphereDiameter * 1.05f;
            float rightEdge = size.x * 0.5f;

            btnResetPosOffset_ = glm::vec3(rightEdge - sphereDiameter * 0.5f - sphereSpacing * 2 + 0.01f, 0.005f, 0.002f);
            btnToggleMinimizeOffset_ = glm::vec3(rightEdge - sphereDiameter * 0.5f - sphereSpacing + 0.01f, 0.005f, 0.002f);
            btnToggleVisibleOffset_ = glm::vec3(rightEdge - sphereDiameter * 0.5f + 0.01f, 0.005f, 0.002f);
            
            btnResetPos_ = std::make_unique<Urbaxio::UI::VRConfirmButtonWidget>(btnResetPosOffset_, sphereDiameter, backIcon, nullptr);
            btnToggleMinimize_ = std::make_unique<Urbaxio::UI::VRConfirmButtonWidget>(btnToggleMinimizeOffset_, sphereDiameter, minIcon, nullptr);
            btnToggleVisible_ = std::make_unique<Urbaxio::UI::VRConfirmButtonWidget>(btnToggleVisibleOffset_, sphereDiameter, closeIcon, nullptr);
            
            btnResetPos_->setDepthEffect(Urbaxio::UI::DepthEffect::CONVEX);
            btnToggleMinimize_->setDepthEffect(Urbaxio::UI::DepthEffect::CONVEX);
            btnToggleVisible_->setDepthEffect(Urbaxio::UI::DepthEffect::CONVEX);
        }

        void Update(const Urbaxio::UI::Ray& localRay, bool triggerPressed, bool triggerReleased, bool triggerHeld, bool aButtonPressed, float stickY) override {
            Urbaxio::UI::VRPanel* panel = manager_.GetPanel(panelName_);
            if (!panel) return;

            // Sync state from the actual panel
            isMinimized_ = panel->IsMinimized();
            isVisible_ = panel->IsVisible();

            const float ANIM_SPEED = 0.15f;
            float targetT = isHovered_ ? 1.0f : 0.0f;
            hoverAnimationT_ += (targetT - hoverAnimationT_) * ANIM_SPEED;
            
            Urbaxio::UI::HitResult hitReset = btnResetPos_->CheckIntersection(localRay);
            Urbaxio::UI::HitResult hitMin = btnToggleMinimize_->CheckIntersection(localRay);
            Urbaxio::UI::HitResult hitClose = btnToggleVisible_->CheckIntersection(localRay);
            
            btnResetPos_->SetHover(hitReset.didHit);
            btnToggleMinimize_->SetHover(hitMin.didHit);
            btnToggleVisible_->SetHover(hitClose.didHit);
            
            if ((triggerPressed || aButtonPressed) && isHovered_) {
                if (hitReset.didHit) panel->ResetPosition();
                if (hitMin.didHit) panel->SetMinimized(!panel->IsMinimized());
                if (hitClose.didHit) panel->SetVisible(!panel->IsVisible());
            }
            
            // Update button colors based on synced state
            btnToggleMinimize_->SetColor(isMinimized_ ? selectedColor_ : defaultColor_);
            btnToggleVisible_->SetColor(isVisible_ ? selectedColor_ : defaultColor_);
            btnResetPos_->SetColor(defaultColor_);
            
            btnResetPos_->Update(localRay, false, false, false, false, 0.0f);
            btnToggleMinimize_->Update(localRay, false, false, false, false, 0.0f);
            btnToggleVisible_->Update(localRay, false, false, false, false, 0.0f);
        }
        
        void Render(Urbaxio::Renderer& renderer, Urbaxio::TextRenderer& textRenderer, const glm::mat4& panelTransform, const glm::mat4& view, const glm::mat4& projection, float alpha, const std::optional<Urbaxio::UI::MaskData>& mask) const override {
            Urbaxio::UI::VRPanel* panel = manager_.GetPanel(panelName_);
            if (!panel) return;
            
            glm::mat4 rowWorldTransform = panelTransform * glm::translate(glm::mat4(1.0f), localPosition_);
            textRenderer.SetPanelModelMatrix(rowWorldTransform);
            
            float textHeight = size_.y * 0.7f;
            glm::vec2 textSize = textRenderer.GetTextSize(panel->GetDisplayName(), textHeight);
            
            float startX = -textSize.x * 0.5f;
            float endX = -size_.x * 0.5f;
            float currentX = glm::mix(startX, endX, hoverAnimationT_);
            
            glm::vec3 textRelativePos(currentX, 0, 0.001f);
            textRenderer.AddTextOnPanel(panel->GetDisplayName(), textRelativePos, glm::vec4(1.0f, 1.0f, 1.0f, alpha), textHeight, Urbaxio::TextAlign::LEFT, mask);
            
            if (hoverAnimationT_ > 0.01f) {
                float sphereAlpha = alpha * hoverAnimationT_;
                std::vector<std::pair<float, const Urbaxio::UI::IVRWidget*>> spheres_to_render;
                for (auto* sphere : {btnResetPos_.get(), btnToggleMinimize_.get(), btnToggleVisible_.get()}) {
                    glm::vec3 sphere_world_pos = panelTransform * glm::vec4(sphere->GetLocalPosition(), 1.0f);
                    float viewZ = (view * glm::vec4(sphere_world_pos, 1.0f)).z;
                    spheres_to_render.emplace_back(viewZ, sphere);
                }

                std::sort(spheres_to_render.begin(), spheres_to_render.end(), [](const auto& a, const auto& b) {
                    return a.first < b.first;
                });

                for (const auto& [depth, sphere] : spheres_to_render) {
                    sphere->Render(renderer, textRenderer, panelTransform, view, projection, sphereAlpha, mask);
                }
            }
        }

        Urbaxio::UI::HitResult CheckIntersection(const Urbaxio::UI::Ray& localRay) override {
            Urbaxio::UI::HitResult result;
            float t;
            if (glm::intersectRayPlane(localRay.origin, localRay.direction, localPosition_, glm::vec3(0, 0, 1), t) && t > 0) {
                glm::vec3 hitPoint = localRay.origin + localRay.direction * t;
                if (glm::abs(hitPoint.x - localPosition_.x) <= size_.x * 0.5f && glm::abs(hitPoint.y - localPosition_.y) <= size_.y * 0.5f) {
                    result.didHit = true;
                    result.distance = t;
                    result.hitWidget = this; 
                }
            }
            return result;
        }
        void HandleClick() override {}
        void SetLocalPosition(const glm::vec3& pos) override {
            localPosition_ = pos;
            btnResetPos_->SetLocalPosition(pos + btnResetPosOffset_);
            btnToggleMinimize_->SetLocalPosition(pos + btnToggleMinimizeOffset_);
            btnToggleVisible_->SetLocalPosition(pos + btnToggleVisibleOffset_);
        }
        void SetHover(bool hover) override { isHovered_ = hover; }
        const glm::vec3& GetLocalPosition() const override { return localPosition_; }
        glm::vec2 GetSize() const override { return size_; }
    private:
        std::string panelName_;
        Urbaxio::UI::VRUIManager& manager_;
        glm::vec2 size_;
        glm::vec3 localPosition_;
        float hoverAnimationT_;
        
        std::unique_ptr<Urbaxio::UI::VRConfirmButtonWidget> btnResetPos_, btnToggleMinimize_, btnToggleVisible_;
        glm::vec3 btnResetPosOffset_, btnToggleMinimizeOffset_, btnToggleVisibleOffset_;
        
        bool isMinimized_;
        bool isVisible_;
        
        const glm::vec3 defaultColor_{0.3f, 0.75f, 1.0f};
        const glm::vec3 selectedColor_{1.0f, 0.79f, 0.4f};
        bool isHovered_ = false;
    };
    // --- END OF MODIFICATION ---
    
    // A widget that wraps a VRScrollWidget and dynamically populates it
    // with a list of all panels from the VRUIManager.
    class VRPanelListWidget : public Urbaxio::UI::IVRWidget {
    public:
        VRPanelListWidget(Urbaxio::UI::VRUIManager& manager, const glm::vec3& localPos, const glm::vec2& size,
                          unsigned int backIcon, unsigned int minIcon, unsigned int closeIcon) 
        : manager_(manager), backIcon_(backIcon), minIcon_(minIcon), closeIcon_(closeIcon)
        {
            scrollWidget_ = std::make_unique<Urbaxio::UI::VRScrollWidget>(localPos, size);
            lastPanelCount_ = 0;
        }
        
        void Update(const Urbaxio::UI::Ray& localRay, bool triggerPressed, bool triggerReleased, bool triggerHeld, bool aButtonPressed, float stickY) override {
            if (manager_.GetPanels().size() != lastPanelCount_) {
                rebuild();
                return;
            }
            scrollWidget_->Update(localRay, triggerPressed, triggerReleased, triggerHeld, aButtonPressed, stickY);
        }
        
        void Render(Urbaxio::Renderer& renderer, Urbaxio::TextRenderer& textRenderer, const glm::mat4& panelTransform, const glm::mat4& view, const glm::mat4& projection, float alpha, const std::optional<Urbaxio::UI::MaskData>& mask) const override {
            scrollWidget_->Render(renderer, textRenderer, panelTransform, view, projection, alpha, mask);
        }
        Urbaxio::UI::HitResult CheckIntersection(const Urbaxio::UI::Ray& localRay) override {
            return scrollWidget_->CheckIntersection(localRay);
        }
        void HandleClick() override {
            scrollWidget_->HandleClick();
        }
        void SetHover(bool hover) override {
            scrollWidget_->SetHover(hover);
        }
        void SetLocalPosition(const glm::vec3& pos) override { scrollWidget_->SetLocalPosition(pos); }
        const glm::vec3& GetLocalPosition() const override { return scrollWidget_->GetLocalPosition(); }
        glm::vec2 GetSize() const override { return scrollWidget_->GetSize(); }
    private:
        // START OF MODIFICATION
        void rebuild() {
            scrollWidget_->ClearChildren();
            scrollWidget_->ClearState();
            glm::vec2 rowSize(0.15f, 0.03f);
            
            for (const auto& [name, panel] : manager_.GetPanels()) {
                if (name == "PanelManager") continue;
                scrollWidget_->AddWidget(std::make_unique<VRPanelRowWidget>(name, manager_, rowSize, backIcon_, minIcon_, closeIcon_));
            }
            lastPanelCount_ = manager_.GetPanels().size();
        }
        // END OF MODIFICATION
        Urbaxio::UI::VRUIManager& manager_;
        std::unique_ptr<Urbaxio::UI::VRScrollWidget> scrollWidget_;
        size_t lastPanelCount_;
        unsigned int backIcon_, minIcon_, closeIcon_;
    };
    // END OF MODIFICATION

    void SetupPanelManagerPanel(Urbaxio::UI::VRUIManager& vruiManager, unsigned int dragIcon, unsigned int closeIcon, unsigned int minimizeIcon, unsigned int backIcon) {
        glm::vec3 translation = glm::vec3(0.004f, 0.045f, 0.020f);
        glm::vec3 eulerAnglesRad = glm::radians(glm::vec3(-117.219f, 2.847f, -4.021f));
        glm::vec3 scale = glm::vec3(0.308f);
        glm::mat4 panelOffset = glm::translate(glm::mat4(1.0f), translation) *
                               glm::mat4_cast(glm::quat(eulerAnglesRad)) *
                               glm::scale(glm::mat4(1.0f), scale);
        
        auto& panelMgr = vruiManager.AddPanel("PanelManager", "Panels", glm::vec2(0.217f, 0.381f), panelOffset, 0.1f, dragIcon, closeIcon, minimizeIcon);
        auto listWidget = std::make_unique<VRPanelListWidget>(vruiManager, glm::vec3(0.0f, -0.01f, 0.01f), glm::vec2(0.18f, 0.3f), backIcon, minimizeIcon, closeIcon);
        panelMgr.AddWidget(std::move(listWidget));
    }

    // Helper to create a model matrix from an OpenXR pose
    glm::mat4 XrPoseToModelMatrix(const XrPosef& pose) {
        glm::quat orientation(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        glm::vec3 position(pose.position.x, pose.position.y, pose.position.z);
        glm::mat4 rotationMatrix = glm::toMat4(orientation);
        glm::mat4 translationMatrix = glm::translate(glm::mat4(1.0f), position);
        return translationMatrix * rotationMatrix;
    }

    // Helper to convert XrPosef (position + quaternion) to a 4x4 view matrix
    glm::mat4 XrPoseToMat4(const XrPosef& pose) {
        glm::quat orientation(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        glm::vec3 position(pose.position.x, pose.position.y, pose.position.z);
        glm::mat4 rotationMatrix = glm::toMat4(orientation);
        glm::mat4 translationMatrix = glm::translate(glm::mat4(1.0f), position);
        // The view matrix is the inverse of the camera's transformation
        return glm::inverse(translationMatrix * rotationMatrix);
    }

    // Transform a 3D point by a 4x4 matrix
    glm::vec3 TransformPoint(const glm::mat4& M, const glm::vec3& p) {
        return glm::vec3(M * glm::vec4(p, 1.0f));
    }

    // Helper to interpolate color from green to cyan
    glm::vec3 MixColorFromPress(float t) {
        return glm::mix(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 1.0f, 1.0f), std::clamp(t, 0.0f, 1.0f));
    }

    // Holds pointers to all SceneObjects that make up a single controller's visual representation.
    struct ControllerVisuals {
        Urbaxio::Engine::SceneObject* mainBody = nullptr;
        Urbaxio::Engine::SceneObject* joystick = nullptr;
        Urbaxio::Engine::SceneObject* triggerButton = nullptr;
        Urbaxio::Engine::SceneObject* grabButton = nullptr;
        Urbaxio::Engine::SceneObject* menuButton = nullptr;
        Urbaxio::Engine::SceneObject* buttonAX = nullptr;
        Urbaxio::Engine::SceneObject* buttonBY = nullptr;
        Urbaxio::Engine::SceneObject* panel = nullptr;
        Urbaxio::Engine::SceneObject* upperDisk = nullptr;
        Urbaxio::Engine::SceneObject* middleDisk = nullptr;
        Urbaxio::Engine::SceneObject* lowerDisk = nullptr;

        // Helper to get all valid part pointers for easier iteration
        std::vector<Urbaxio::Engine::SceneObject*> GetAllParts() const {
            std::vector<Urbaxio::Engine::SceneObject*> parts;
            // A macro to reduce boilerplate
            #define ADD_PART(p) if (p) parts.push_back(p)
            ADD_PART(mainBody);
            ADD_PART(joystick);
            ADD_PART(triggerButton);
            ADD_PART(grabButton);
            ADD_PART(menuButton);
            ADD_PART(buttonAX);
            ADD_PART(buttonBY);
            ADD_PART(panel);
            ADD_PART(upperDisk);
            ADD_PART(middleDisk);
            ADD_PART(lowerDisk);
            #undef ADD_PART
            return parts;
        }
    };

    // Helper function to load all OBJ parts for a single controller and create SceneObjects
    void CreateControllerPartObjects(Urbaxio::Engine::Scene* scene, ControllerVisuals& visuals, const std::string& handPrefix) {
        if (!scene) return;
        
        // Map internal names to file names and struct members
        const std::map<std::string, std::pair<std::string, Urbaxio::Engine::SceneObject**>> partMap = {
            { "mainBody",     { "controller_mainBody.obj",        &visuals.mainBody } },
            { "joystick",     { "controller_joystick.obj",        &visuals.joystick } },
            { "button_trigger", { "controller_button_trigger.obj",  &visuals.triggerButton } },
            { "button_grab",  { "controller_button_grab.obj",     &visuals.grabButton } },
            { "button_menu",  { "controller_button_menu.obj",     &visuals.menuButton } },
            { "button_AX",    { "controller_button_AX.obj",       &visuals.buttonAX } },
            { "button_BY",    { "controller_button_BY.obj",       &visuals.buttonBY } },
            { "panel",        { "controller_panel.obj",           &visuals.panel } },
            { "upper_disk",   { "controller_upper_disk.obj",      &visuals.upperDisk } },
            { "middle_disk",  { "controller_middle_disk.obj",     &visuals.middleDisk } },
            { "lower_disk",   { "controller_lower_disk.obj",     &visuals.lowerDisk } },
        };

        for (const auto& [name, data] : partMap) {
            std::string objPath = "../../resources/" + data.first;
            if (!std::filesystem::exists(objPath)) {
                objPath = "../../../resources/" + data.first;
            }

            Urbaxio::CadKernel::MeshBuffers partMesh = Urbaxio::FileIO::LoadMeshFromObj(objPath);
            if (!partMesh.isEmpty()) {
                std::string objectName = handPrefix + "_" + name;
                Urbaxio::Engine::SceneObject* newPart = scene->create_object(objectName);
                if (newPart) {
                    // NEW: Use the new geometry system for mesh-only objects
                    newPart->setGeometry(std::make_unique<Urbaxio::Engine::MeshGeometry>(std::move(partMesh)));
                    newPart->setExportable(false);
                    *(data.second) = newPart; // Assign the created object pointer to the struct member
                }
            } else {
                std::cerr << "Shell Warning: Could not load controller part model from " << objPath << std::endl;
            }
        }
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

    void RenderLoadingPopup(Urbaxio::LoadingManager& manager) {
        // Only show blocking popup for voxelization and file loading (NOT for remeshing!)
        if (manager.IsBlockingOperation()) {
            ImGui::OpenPopup("Loading...");
        }

        ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
        ImGui::SetNextWindowSize(ImVec2(350, 0));
        if (ImGui::BeginPopupModal("Loading...", NULL, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings)) {
            ImGui::Text("%s", manager.GetStatus().c_str());
            ImGui::ProgressBar(manager.GetProgress(), ImVec2(-1, 0));

            if (!manager.IsBlockingOperation()) {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }
        
        // Show small non-blocking indicator for background operations (remeshing)
        if (manager.IsLoading() && !manager.IsBlockingOperation()) {
            ImGuiIO& io = ImGui::GetIO();
            ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - 10, 10), ImGuiCond_Always, ImVec2(1.0f, 0.0f));
            ImGui::SetNextWindowBgAlpha(0.7f);
            if (ImGui::Begin("BackgroundTask", nullptr, 
                ImGuiWindowFlags_NoDecoration | 
                ImGuiWindowFlags_NoMove | 
                ImGuiWindowFlags_NoSavedSettings |
                ImGuiWindowFlags_AlwaysAutoResize)) {
                ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.5f, 1.0f), "Remeshing...");
            }
            ImGui::End();
        }
    }

    void RenderImportOptionsPopup(
        bool& show, 
        const std::string& filepath, 
        Urbaxio::LoadingManager& loadingManager)
    {
        if (show) {
            ImGui::OpenPopup("Import Options");
            show = false;
        }

        ImGui::SetNextWindowSize(ImVec2(350, 180), ImGuiCond_FirstUseEver);
        if (ImGui::BeginPopupModal("Import Options", NULL, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::Text("Importing: %s", std::filesystem::path(filepath).filename().string().c_str());
            ImGui::Separator();

            static int selected_unit = 2;
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
                loadingManager.RequestLoadObj(filepath, custom_scale);
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

    bool LoadProject(const std::string& path, Urbaxio::Engine::Scene* scene, Urbaxio::Camera& camera, Urbaxio::Renderer& renderer) { // --- NEW: Pass renderer
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
        bool result = scene->LoadFromStream(file);
        
        // --- NEW: Invalidate renderer's static batch ---
        renderer.InvalidateStaticBatch();
        
        return result;
    }

    void RecreateEssentialMarkers(Urbaxio::Engine::Scene* scene, float capsuleRadius, float capsuleHeight10m, float capsuleHeight5m) {
        if (!scene) return;
        // Center Marker
        Urbaxio::Engine::SceneObject* center_sphere = scene->create_object("CenterMarker");
        if (center_sphere) {
            center_sphere->setGeometry(std::make_unique<Urbaxio::Engine::MeshGeometry>(CreateIcoSphereMesh(0.25f, 2)));
            center_sphere->setExportable(false); // Do not export this
        }
        // Capsule Markers
        Urbaxio::Engine::SceneObject* cap10 = scene->create_object("UnitCapsuleMarker10m");
        if (cap10) {
            cap10->setGeometry(std::make_unique<Urbaxio::Engine::MeshGeometry>(CreateCapsuleMesh(capsuleRadius, capsuleHeight10m)));
            cap10->setExportable(false); // Do not export this
        }
        Urbaxio::Engine::SceneObject* cap5 = scene->create_object("UnitCapsuleMarker5m");
        if (cap5) {
            cap5->setGeometry(std::make_unique<Urbaxio::Engine::MeshGeometry>(CreateCapsuleMesh(capsuleRadius, capsuleHeight5m)));
            cap5->setExportable(false); // Do not export this
        }
    }

    // --- NEW: Factory function to create our VR panels ---
    void SetupVRPanels(Urbaxio::UI::VRUIManager& vruiManager, std::string& numpadInputTarget, Urbaxio::Tools::ToolManager& toolManager, bool& isNumpadActiveFlag, const Urbaxio::Tools::ToolContext& toolContext, unsigned int dragIconTexture, unsigned int closeIconTexture, unsigned int minimizeIconTexture) {
        {
            // --- Transform values from your tuning ---
            glm::vec3 translation = glm::vec3(-0.012f, -0.063f, -0.127f);
            glm::vec3 eulerAnglesRad = glm::radians(glm::vec3(-109.914f, 3.989f, -3.402f));
            glm::vec3 scale = glm::vec3(0.297f, 0.297f, 0.297f);

            glm::mat4 offset = glm::translate(glm::mat4(1.0f), translation) *
                               glm::mat4_cast(glm::quat(eulerAnglesRad)) *
                               glm::scale(glm::mat4(1.0f), scale);
            
            // --- Constructor parameters from your tuning ---
            auto& numpad = vruiManager.AddPanel("NewNumpad", "Numpad", glm::vec2(0.226f, 0.427f), offset, 0.1f, dragIconTexture, closeIconTexture, minimizeIconTexture);

            glm::vec2 displaySize(0.14f, 0.05f); 
            numpad.AddWidget(std::make_unique<Urbaxio::UI::VRDisplayWidget>(glm::vec3(0), displaySize, numpadInputTarget));
            
            // --- FIX: Implement Confirm Callback ---
            auto confirmCallback = [&]() {
                if (numpadInputTarget.empty() || numpadInputTarget == ".") return;

                using Urbaxio::Tools::ToolType;
                auto* tool = toolManager.GetActiveTool();
                if (!tool) return;

                if (toolManager.GetActiveToolType() == ToolType::Line) {
                    auto* line = static_cast<Urbaxio::Tools::LineTool*>(tool);
                    line->SetLengthInput(numpadInputTarget);
                    tool->OnKeyDown(SDLK_RETURN, false, false);
                } else if (toolManager.GetActiveToolType() == ToolType::PushPull) {
                    auto* pp = static_cast<Urbaxio::Tools::PushPullTool*>(tool);
                    pp->SetLengthInput(numpadInputTarget);
                    tool->OnKeyDown(SDLK_RETURN, false, false);
                }
                
                isNumpadActiveFlag = false;
            };
            float topButtonSize = 0.05f;
            numpad.AddWidget(std::make_unique<Urbaxio::UI::VRConfirmButtonWidget>(glm::vec3(0), topButtonSize, glm::vec3(0.1f, 0.8f, 0.2f), confirmCallback));

            float keyTextHeight = 0.03f;
            const char* keys[] = {"1","2","3", "4","5","6", "7","8","9", ".","0","<-"};
            for (int i = 0; i < 12; ++i) {
                std::string keyStr = keys[i];
                
                // --- FIX: Implement Key Callback ---
                auto callback = [keyStr, &numpadInputTarget, &isNumpadActiveFlag]() {
                    if (!isNumpadActiveFlag) {
                        numpadInputTarget = "0";
                        isNumpadActiveFlag = true;
                    }

                    if (keyStr == "<-") {
                        if (!numpadInputTarget.empty()) numpadInputTarget.pop_back();
                        if (numpadInputTarget.empty()) numpadInputTarget = "0";
                        return;
                    }

                    if (keyStr == ".") {
                        if (numpadInputTarget.find('.') == std::string::npos) {
                            if (numpadInputTarget.empty()) numpadInputTarget = "0";
                            numpadInputTarget.push_back('.');
                        }
                        return;
                    }
                    
                    if (numpadInputTarget == "0") numpadInputTarget.clear();
                    numpadInputTarget.push_back(keyStr[0]);
                };
                numpad.AddWidget(std::make_unique<Urbaxio::UI::VRButtonWidget>(keyStr, glm::vec3(0), glm::vec2(keyTextHeight), callback));
            }
            
            // --- Position all widgets ---
            const float contentOffsetY = -0.02f;
            const float topRowY = 0.17f + contentOffsetY;
            
            numpad.GetWidget(0)->SetLocalPosition({-0.035f, topRowY, 0.01f}); // Display
            numpad.GetWidget(1)->SetLocalPosition({0.07f, topRowY, 0.01f});   // Confirm button
            
            float keySpacing = 0.065f;
            for (int i = 0; i < 12; ++i) {
                glm::vec3 keyCenter(0);
                if (i < 9) keyCenter = glm::vec3(((float)(i % 3) - 1.0f) * keySpacing, 0.09f - (float)(i / 3) * keySpacing + contentOffsetY, 0.01f);
                else if (i == 9) keyCenter = glm::vec3(-1.0f * keySpacing, 0.09f - 3.0f * keySpacing + contentOffsetY, 0.01f);
                else if (i == 10) keyCenter = glm::vec3(0.0f, 0.09f - 3.0f * keySpacing + contentOffsetY, 0.01f);
                else if (i == 11) keyCenter = glm::vec3(1.0f * keySpacing, 0.09f - 3.0f * keySpacing + contentOffsetY, 0.01f);
                numpad.GetWidget(i + 2)->SetLocalPosition(keyCenter);
            }
        }
    }

    // --- NEW: Factory function to create the tool menu panel ---
    void SetupStandardToolsPanel(Urbaxio::UI::VRUIManager& vruiManager, Urbaxio::Tools::ToolManager& toolManager, unsigned int dragIconTexture, unsigned int selectIcon, unsigned int lineIcon, unsigned int pushpullIcon, unsigned int moveIcon, unsigned int paintIcon, unsigned int closeIconTexture, unsigned int minimizeIconTexture) {
        // --- FINAL: Default transform values from your tuning ---
        glm::vec3 translation = glm::vec3(0.059f, -0.033f, -0.050f);
        glm::vec3 eulerAnglesRad = glm::radians(glm::vec3(-113.214f, 3.417f, -3.030f));
        glm::vec3 scale = glm::vec3(0.403f, 0.403f, 0.403f);

        glm::mat4 panelOffset = glm::translate(glm::mat4(1.0f), translation) *
                                glm::mat4_cast(glm::quat(eulerAnglesRad)) *
                                glm::scale(glm::mat4(1.0f), scale);
        
        // --- FINAL: Default constructor parameters from your tuning ---
        float panelWidth = 0.064f;
        float panelHeight = 0.394f;
        float cornerRadius = 0.5f;

        auto& toolMenu = vruiManager.AddPanel("StandardTools", "Standard", glm::vec2(panelWidth, panelHeight), panelOffset, cornerRadius, dragIconTexture, closeIconTexture, minimizeIconTexture);
        
        // Adjust layout for 5 buttons
        toolMenu.SetLayout(std::make_unique<Urbaxio::UI::VerticalLayout>(0.015f, true)); // Reduced spacing a bit

        toolMenu.AddWidget(std::make_unique<Urbaxio::UI::VRToolButtonWidget>("Select", glm::vec3(0), glm::vec2(0), selectIcon, Urbaxio::Tools::ToolType::Select, toolManager, [&toolManager]() { toolManager.SetTool(Urbaxio::Tools::ToolType::Select); }));
        toolMenu.AddWidget(std::make_unique<Urbaxio::UI::VRToolButtonWidget>("Line", glm::vec3(0), glm::vec2(0), lineIcon, Urbaxio::Tools::ToolType::Line, toolManager, [&toolManager]() { toolManager.SetTool(Urbaxio::Tools::ToolType::Line); }));
        toolMenu.AddWidget(std::make_unique<Urbaxio::UI::VRToolButtonWidget>("Push/Pull", glm::vec3(0), glm::vec2(0), pushpullIcon, Urbaxio::Tools::ToolType::PushPull, toolManager, [&toolManager]() { toolManager.SetTool(Urbaxio::Tools::ToolType::PushPull); }));
        toolMenu.AddWidget(std::make_unique<Urbaxio::UI::VRToolButtonWidget>("Move", glm::vec3(0), glm::vec2(0), moveIcon, Urbaxio::Tools::ToolType::Move, toolManager, [&toolManager]() { toolManager.SetTool(Urbaxio::Tools::ToolType::Move); }));
        toolMenu.AddWidget(std::make_unique<Urbaxio::UI::VRToolButtonWidget>("Paint", glm::vec3(0), glm::vec2(0), paintIcon, Urbaxio::Tools::ToolType::Paint, toolManager, [&toolManager]() { toolManager.SetTool(Urbaxio::Tools::ToolType::Paint); }));

        toolMenu.RecalculateLayout();
    }

    void SetupSculptToolsPanel(Urbaxio::UI::VRUIManager& vruiManager, Urbaxio::Tools::ToolManager& toolManager, const Urbaxio::Tools::ToolContext& toolContext, unsigned int dragIcon, unsigned int sculptIcon, unsigned int sculptDrawIcon, unsigned int sculptPinchIcon, unsigned int sculptSmoothIcon, unsigned int voxelizationIcon, unsigned int closeIcon, unsigned int minimizeIcon) {
        // Position it to the right of the standard tools panel
        glm::vec3 translation = glm::vec3(0.096f, -0.035f, -0.052f);
        glm::vec3 eulerAnglesRad = glm::radians(glm::vec3(-113.341f, 2.228f, -6.367f));
        glm::vec3 scale = glm::vec3(0.403f, 0.403f, 0.403f);

        glm::mat4 panelOffset = glm::translate(glm::mat4(1.0f), translation) *
                                glm::mat4_cast(glm::quat(eulerAnglesRad)) *
                                glm::scale(glm::mat4(1.0f), scale);

        // Adjust panel size for 6 buttons (with GPU Sculpt)
        float panelWidth = 0.064f;
#ifdef URBAXIO_GPU_ENABLED
#if URBAXIO_GPU_ENABLED
        float panelHeight = 0.458f; // Increased for GPU Sculpt button
#else
        float panelHeight = 0.394f; // 5 buttons
#endif
#else
        float panelHeight = 0.394f; // 5 buttons
#endif
        float cornerRadius = 0.5f;

        auto& sculptMenu = vruiManager.AddPanel("SculptureTools", "Sculpture", glm::vec2(panelWidth, panelHeight), panelOffset, cornerRadius, dragIcon, closeIcon, minimizeIcon);
        
        sculptMenu.SetLayout(std::make_unique<Urbaxio::UI::VerticalLayout>(0.015f, true)); // Same layout

        // Voxelize Action Button Callback
        auto voxelizeCallback = [&toolContext]() {
            if (toolContext.selectedObjId && *toolContext.selectedObjId != 0 && toolContext.scene && toolContext.loadingManager) {
                Urbaxio::Engine::SceneObject* obj = toolContext.scene->get_object_by_id(*toolContext.selectedObjId);
                if (obj && dynamic_cast<Urbaxio::Engine::BRepGeometry*>(obj->getGeometry())) {
                    // Using a fixed resolution for now, can be made configurable later
                    toolContext.loadingManager->RequestVoxelize(toolContext.scene, *toolContext.selectedObjId, 128);
                }
            }
        };
        
        // Placeholder for unimplemented tools
        auto placeholderCallback = [](){ std::cout << "VR UI: This sculpt tool is not yet implemented." << std::endl; };

        // Define the custom color themes for the sculpt panel
        const Urbaxio::UI::ToolButtonColors sculptInactiveColors = { 
            {0.04f, 0.89f, 0.03f}, // base
            {0.68f, 0.47f, 0.99f}, // aberration1
            {0.59f, 0.99f, 0.62f}  // aberration2
            
        };
        const Urbaxio::UI::ToolButtonColors sculptSelectedColors = { 
            {0.96f, 0.22f, 0.29f}, // base
            {0.99f, 0.29f, 0.66f}, // aberration1
            {0.99f, 0.46f, 0.29f}  // aberration2
        };

        // Add buttons to the panel, passing the new color themes
        sculptMenu.AddWidget(std::make_unique<Urbaxio::UI::VRToolButtonWidget>("Sculpt", glm::vec3(0), glm::vec2(0), sculptIcon, Urbaxio::Tools::ToolType::Sculpt, toolManager, [&toolManager]() { toolManager.SetTool(Urbaxio::Tools::ToolType::Sculpt); }, sculptSelectedColors, sculptInactiveColors));
        
#ifdef URBAXIO_GPU_ENABLED
#if URBAXIO_GPU_ENABLED
        // GPU-accelerated sculpt tool (cyan colors to distinguish from CPU version)
        const Urbaxio::UI::ToolButtonColors gpuInactiveColors = { 
            {0.03f, 0.89f, 0.89f}, // base (cyan)
            {0.47f, 0.68f, 0.99f}, // aberration1
            {0.62f, 0.99f, 0.89f}  // aberration2
        };
        const Urbaxio::UI::ToolButtonColors gpuSelectedColors = { 
            {0.22f, 0.96f, 0.96f}, // base (bright cyan)
            {0.29f, 0.99f, 0.66f}, // aberration1
            {0.46f, 0.99f, 0.99f}  // aberration2
        };
        sculptMenu.AddWidget(std::make_unique<Urbaxio::UI::VRToolButtonWidget>("GPU Sculpt", glm::vec3(0), glm::vec2(0), sculptIcon, Urbaxio::Tools::ToolType::SculptGpu, toolManager, [&toolManager]() { toolManager.SetTool(Urbaxio::Tools::ToolType::SculptGpu); }, gpuSelectedColors, gpuInactiveColors));
#endif
#endif
        
        sculptMenu.AddWidget(std::make_unique<Urbaxio::UI::VRToolButtonWidget>("Draw", glm::vec3(0), glm::vec2(0), sculptDrawIcon, Urbaxio::Tools::ToolType::SculptDraw, toolManager, [&toolManager]() { toolManager.SetTool(Urbaxio::Tools::ToolType::SculptDraw); std::cout << "VR UI: This sculpt tool is not yet implemented." << std::endl; }, sculptSelectedColors, sculptInactiveColors));
        sculptMenu.AddWidget(std::make_unique<Urbaxio::UI::VRToolButtonWidget>("Pinch", glm::vec3(0), glm::vec2(0), sculptPinchIcon, Urbaxio::Tools::ToolType::SculptPinch, toolManager, [&toolManager]() { toolManager.SetTool(Urbaxio::Tools::ToolType::SculptPinch); std::cout << "VR UI: This sculpt tool is not yet implemented." << std::endl; }, sculptSelectedColors, sculptInactiveColors));
        sculptMenu.AddWidget(std::make_unique<Urbaxio::UI::VRToolButtonWidget>("Smooth", glm::vec3(0), glm::vec2(0), sculptSmoothIcon, Urbaxio::Tools::ToolType::SculptSmooth, toolManager, [&toolManager]() { toolManager.SetTool(Urbaxio::Tools::ToolType::SculptSmooth); std::cout << "VR UI: This sculpt tool is not yet implemented." << std::endl; }, sculptSelectedColors, sculptInactiveColors));
        sculptMenu.AddWidget(std::make_unique<Urbaxio::UI::VRToolButtonWidget>("Voxelize", glm::vec3(0), glm::vec2(0), voxelizationIcon, Urbaxio::Tools::ToolType::VoxelizeAction, toolManager, voxelizeCallback, sculptSelectedColors, sculptInactiveColors));
        
        sculptMenu.RecalculateLayout();
    }

    void SetupFileMenuPanel(Urbaxio::UI::VRUIManager& vruiManager, unsigned int dragIcon, unsigned int closeIcon, unsigned int minimizeIcon, SDL_Window* window, std::atomic<bool>& isFileDialogActive, std::atomic<bool>& fileDialogResultReady, std::string& filePathFromDialog, std::mutex& filePathMutex, bool& isImportDialog) {
        glm::vec3 translation = glm::vec3(-0.076f, -0.030f, -0.063f);
        glm::vec3 eulerAnglesRad = glm::radians(glm::vec3(-114.529f, 1.170f, -11.124f));
        glm::vec3 scale = glm::vec3(0.365f);

        glm::mat4 panelOffset = glm::translate(glm::mat4(1.0f), translation) *
                                glm::mat4_cast(glm::quat(eulerAnglesRad)) *
                                glm::scale(glm::mat4(1.0f), scale);

        auto& fileMenu = vruiManager.AddPanel("FileMenu", "File", glm::vec2(0.156f, 0.293f), panelOffset, 0.1f, dragIcon, closeIcon, minimizeIcon);

        auto importCallback = [&, window]() {
            if (isFileDialogActive.load()) return;
            isFileDialogActive = true;
            fileDialogResultReady = false;
            isImportDialog = true;

            std::thread([&, window]() {
                std::string path = OpenObjDialog(window);
                {
                    std::lock_guard<std::mutex> lock(filePathMutex);
                    filePathFromDialog = path;
                }
                fileDialogResultReady = true;
                isFileDialogActive = false;
            }).detach();
        };

        auto exportCallback = [&, window]() {
            if (isFileDialogActive.load()) return;
            isFileDialogActive = true;
            fileDialogResultReady = false;
            isImportDialog = false;

            std::thread([&, window]() {
                std::string path = SaveObjDialog(window);
                {
                    std::lock_guard<std::mutex> lock(filePathMutex);
                    filePathFromDialog = path;
                }
                fileDialogResultReady = true;
                isFileDialogActive = false;
            }).detach();
        };

        fileMenu.AddWidget(std::make_unique<Urbaxio::UI::VRButtonWidget>("Import...", glm::vec3(0), glm::vec2(0.18f, 0.04f), importCallback));
        fileMenu.AddWidget(std::make_unique<Urbaxio::UI::VRButtonWidget>("Export...", glm::vec3(0), glm::vec2(0.18f, 0.04f), exportCallback));

        fileMenu.SetLayout(std::make_unique<Urbaxio::UI::VerticalLayout>(0.01f));
        fileMenu.RecalculateLayout();
    }

    // --- START OF MODIFICATION ---
    unsigned int LoadTextureFromFile(const std::string& path, GLint wrapMode = GL_REPEAT) {
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

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapMode);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapMode);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        stbi_image_free(data);
        return textureID;
    }
    // --- END OF MODIFICATION ---

} // end anonymous namespace

// --- GPU Mesh Upload Helper ---
bool UploadMeshToGPU(Urbaxio::Engine::SceneObject& object) {
    // NEW: Use the caching getter
    const Urbaxio::CadKernel::MeshBuffers& mesh = object.getMeshBuffers();
    if (mesh.isEmpty() || mesh.normals.empty()) {
        if (mesh.normals.empty() && !mesh.vertices.empty()) {
            std::cerr << "UploadMeshToGPU: Mesh for object " << object.get_id() << " is missing normals!" << std::endl;
        }
        return false;
    }
    
    // Clean up old buffers if they exist
    if (object.vao != 0) glDeleteVertexArrays(1, &object.vao);
    if (object.vbo_vertices != 0) glDeleteBuffers(1, &object.vbo_vertices);
    if (object.vbo_normals != 0) glDeleteBuffers(1, &object.vbo_normals);
    if (object.vbo_uvs != 0) glDeleteBuffers(1, &object.vbo_uvs);
    if (object.ebo != 0) glDeleteBuffers(1, &object.ebo);
    object.vao = object.vbo_vertices = object.vbo_normals = object.vbo_uvs = object.ebo = 0;
    object.index_count = 0;

    glGenVertexArrays(1, &object.vao);
    if (object.vao == 0) return false;
    glBindVertexArray(object.vao);

    // VBO for vertices
    glGenBuffers(1, &object.vbo_vertices);
    if (object.vbo_vertices == 0) { glDeleteVertexArrays(1, &object.vao); object.vao = 0; return false; }
    glBindBuffer(GL_ARRAY_BUFFER, object.vbo_vertices);
    glBufferData(GL_ARRAY_BUFFER, mesh.vertices.size() * sizeof(float), mesh.vertices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // VBO for normals
    glGenBuffers(1, &object.vbo_normals);
    if (object.vbo_normals == 0) { /* cleanup */ return false; }
    glBindBuffer(GL_ARRAY_BUFFER, object.vbo_normals);
    glBufferData(GL_ARRAY_BUFFER, mesh.normals.size() * sizeof(float), mesh.normals.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);

    // --- NEW: VBO for UVs (if they exist) ---
    if (!mesh.uvs.empty()) {
        glGenBuffers(1, &object.vbo_uvs);
        if (object.vbo_uvs == 0) { /* cleanup */ return false; }
        glBindBuffer(GL_ARRAY_BUFFER, object.vbo_uvs);
        glBufferData(GL_ARRAY_BUFFER, mesh.uvs.size() * sizeof(float), mesh.uvs.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(2);
    }

    // EBO for indices
    glGenBuffers(1, &object.ebo);
    if (object.ebo == 0) { /* cleanup */ return false; }
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, object.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh.indices.size() * sizeof(unsigned int), mesh.indices.data(), GL_STATIC_DRAW);
    object.index_count = static_cast<GLsizei>(mesh.indices.size());

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    std::cout << "UploadMeshToGPU: Successfully uploaded mesh for object " << object.get_id() << std::endl;
    return true;
}

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
    Urbaxio::LoadingManager loadingManager;
    
    // --- Brush Cursor Object & Material ---
    Urbaxio::Engine::Material brushMat;
    brushMat.name = "BrushCursorMaterial";
    brushMat.diffuseColor = glm::vec3(1.0f, 0.6f, 0.0f); // Orange
    scene_ptr->getMaterialManager()->AddMaterial(brushMat);

    Urbaxio::Engine::SceneObject* brushCursor = scene_ptr->create_object("BrushCursor");
    if(brushCursor) {
        brushCursor->setGeometry(std::make_unique<Urbaxio::Engine::MeshGeometry>(CreateIcoSphereMesh(1.0f, 2)));
        brushCursor->setExportable(false);
        // Assign material by creating a mesh group
        brushCursor->meshGroups.clear();
        brushCursor->meshGroups.push_back({"BrushCursorMaterial", 0, brushCursor->getMeshBuffers().indices.size()});
    }
    
    // --- Appearance Settings ---
    ImVec4 clear_color = ImVec4(0.13f, 0.13f, 0.18f, 1.00f);
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
    bool show_material_editor = true; // <-- NEW: Show by default

    // --- Core State Variables ---
    uint64_t selectedObjId = 0;
    std::vector<size_t> selectedTriangleIndices;
    std::set<uint64_t> selectedLineIDs;
    glm::vec3 selectionHighlightColor = glm::vec3(1.0f, 225.0f / 255.0f, 84.0f / 255.0f);

    uint64_t hoveredObjId = 0;
    std::vector<size_t> hoveredFaceTriangleIndices;
    glm::vec3 hoverHighlightColor = glm::vec3(0.4f, 0.9f, 1.0f); // Light cyan
    
    // --- NEW: State for the menu sphere WIDGET ---
    std::unique_ptr<Urbaxio::UI::VRConfirmButtonWidget> menuSphereWidget = nullptr; // Меняем тип на кнопку с иконкой
    glm::mat4 menuSphereOffset = glm::translate(glm::mat4(1.0f), glm::vec3(-0.00645685f, -0.0384107f, -0.0818305f)); // Новое смещение
    bool isGrabbingMenuSphere = false;
    glm::mat4 grabbedInitialSphereWorldTransform; // Правильная переменная для хранения состояния захвата
    glm::mat4 grabbedControllerInitialTransform;
    const float menuSphereDiameter = 0.025f; // Уменьшенный в два раза диаметр
    
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
    // --- START OF MODIFICATION ---
    toolContext.isVrMode = vr_mode;
    // --- END OF MODIFICATION ---
    toolContext.loadingManager = &loadingManager; // For async remeshing
    toolContext.meshManager = scene_ptr->getMeshManager();
    if (vrManager) {
        toolContext.worldTransform = &vrManager->GetWorldTransform();
        toolContext.rightThumbstickY = &vrManager->rightThumbstickY;
    }
    
    Urbaxio::Tools::ToolManager toolManager(toolContext);
    toolManager.AddTool(Urbaxio::Tools::ToolType::Paint, std::make_unique<Urbaxio::Tools::PaintTool>()); // <-- NEW
    toolManager.AddTool(Urbaxio::Tools::ToolType::Sculpt, std::make_unique<Urbaxio::Tools::SculptTool>());

    // --- Marker settings ---
    static float capsuleRadius = 0.5f;
    static float capsuleHeight10m = 3.2f;
    static float capsuleHeight5m = 1.4f;

    // --- Create all essential markers ---
    RecreateEssentialMarkers(scene_ptr, capsuleRadius, capsuleHeight10m, capsuleHeight5m);

    

    // --- NEW: Create VR Controller Visuals ---
    ControllerVisuals leftController;
    ControllerVisuals rightController;
    if (vr_mode) {
        CreateControllerPartObjects(scene_ptr, leftController, "Left");
        CreateControllerPartObjects(scene_ptr, rightController, "Right");
        std::cout << "Shell: Successfully loaded multi-part controller models." << std::endl;
    }
    // --- NEW: Live tuning variables for controller model offset, initialized with tuned values ---
    static glm::vec3 g_controllerOffsetTranslate(0.012f, 0.002f, 0.006f);
    static glm::vec3 g_controllerOffsetEuler(273.000f, 166.500f, 81.000f);
    static float g_controllerModelScale = 0.7814f;
    

    // --- NEW: VR UI Manager ---
    Urbaxio::UI::VRUIManager vruiManager;
    std::string g_newNumpadInput = "0";

    std::atomic<bool> isFileDialogActive = false;
    std::atomic<bool> fileDialogResultReady = false;
    std::string filePathFromDialog;
    std::mutex filePathMutex;
    bool isImportDialog = true;

    // --- NEW: Load UI Icons ---
    auto load_icon = [](const std::string& name) -> unsigned int {
        std::string path = "../../resources/" + name;
        if (!std::filesystem::exists(path)) {
            path = "../../../resources/" + name;
        }
        // --- START OF MODIFICATION ---
        return LoadTextureFromFile(path, GL_CLAMP_TO_EDGE);
        // --- END OF MODIFICATION ---
    };
    unsigned int dragIconTexture = load_icon("drag_icon.png");
    unsigned int selectIconTexture = load_icon("select_icon.png");
    unsigned int lineIconTexture = load_icon("line_icon.png");
    unsigned int pushpullIconTexture = load_icon("pushpull_icon.png");
    unsigned int moveIconTexture = load_icon("move_icon.png");
    unsigned int paintBucketIconTexture = load_icon("paint_bucket_icon.png");
    unsigned int sculptIconTexture = load_icon("clay_two_hands.png");
    unsigned int sculptDrawIconTexture = load_icon("clay_control.png");
    unsigned int sculptPinchIconTexture = load_icon("clay_pinch.png");
    unsigned int sculptSmoothIconTexture = load_icon("clay_smooth.png");
    unsigned int voxelizationIconTexture = load_icon("voxelization.png");
    unsigned int closeIconTexture = load_icon("close_icon.png");
    unsigned int minimizeIconTexture = load_icon("minimize_icon.png");
    unsigned int panelManagerIconTexture = load_icon("panel_manager.png");
    unsigned int backIconTexture = load_icon("back.png");

    // --- NEW: Setup our VR panels using the new system ---
    SetupVRPanels(vruiManager, g_newNumpadInput, toolManager, numpadInputActive, toolContext, dragIconTexture, closeIconTexture, minimizeIconTexture);
    SetupStandardToolsPanel(vruiManager, toolManager, dragIconTexture, selectIconTexture, lineIconTexture, pushpullIconTexture, moveIconTexture, paintBucketIconTexture, closeIconTexture, minimizeIconTexture);
    SetupSculptToolsPanel(vruiManager, toolManager, toolContext, dragIconTexture, sculptIconTexture, sculptDrawIconTexture, sculptPinchIconTexture, sculptSmoothIconTexture, voxelizationIconTexture, closeIconTexture, minimizeIconTexture);
    SetupPanelManagerPanel(vruiManager, dragIconTexture, closeIconTexture, minimizeIconTexture, backIconTexture);
    SetupFileMenuPanel(vruiManager, dragIconTexture, closeIconTexture, minimizeIconTexture, window, isFileDialogActive, fileDialogResultReady, filePathFromDialog, filePathMutex, isImportDialog);
    
    if (auto* panelMgr = vruiManager.GetPanel("PanelManager")) {
        panelMgr->SetVisibilityMode(Urbaxio::UI::VisibilityMode::TOGGLE_VIA_FLAG);
        panelMgr->SetVisible(false); // Изначально она выключена
    }

    // --- NEW: Create the menu sphere WIDGET ---
    menuSphereWidget = std::make_unique<Urbaxio::UI::VRConfirmButtonWidget>(
        glm::vec3(0.0f), // Локальная позиция (центр)
        menuSphereDiameter,
        panelManagerIconTexture, // Передаём ID иконки
        [&vruiManager](){
            if (auto* panelMgr = vruiManager.GetPanel("PanelManager")) {
                panelMgr->SetVisible(!panelMgr->IsVisible());
            }
        },
        glm::vec3(1.0f) // Базовый цвет (белый)
    );
    if (menuSphereWidget) {
        menuSphereWidget->SetIconOffsetUseCameraForward(true);
        // Сделать глубину у шарика открытия менеджера панелей более выраженной
        menuSphereWidget->SetDepthStrength(1.8f, 1.8f);
    }

    // --- NEW: State for alternate VR click ---
    bool rightAButtonWasPressed = false;

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

        // --- Undo/Redo Shortcuts (Desktop Mode) ---
        if (!vr_mode && ctrlDown) {
            if (keyboardState[SDL_SCANCODE_Z] && !keyboardState[SDL_SCANCODE_LSHIFT] && !keyboardState[SDL_SCANCODE_RSHIFT]) {
                static bool undoKeyWasPressed = false;
                if (!undoKeyWasPressed) {
                    scene_ptr->getCommandManager()->Undo();
                    // Request async remesh for volumetric objects after Undo
                    for (auto* obj : scene_ptr->get_all_objects()) {
                        if (obj && dynamic_cast<Urbaxio::Engine::VolumetricGeometry*>(obj->getGeometry())) {
                            loadingManager.RequestRemesh(scene_ptr, obj->get_id());
                        }
                    }
                    undoKeyWasPressed = true;
                }
            } else {
                static bool undoKeyWasPressed = false;
                undoKeyWasPressed = false;
            }
            
            if ((keyboardState[SDL_SCANCODE_Y] || (keyboardState[SDL_SCANCODE_Z] && (keyboardState[SDL_SCANCODE_LSHIFT] || keyboardState[SDL_SCANCODE_RSHIFT])))) {
                static bool redoKeyWasPressed = false;
                if (!redoKeyWasPressed) {
                    scene_ptr->getCommandManager()->Redo();
                    // Request async remesh for volumetric objects after Redo
                    for (auto* obj : scene_ptr->get_all_objects()) {
                        if (obj && dynamic_cast<Urbaxio::Engine::VolumetricGeometry*>(obj->getGeometry())) {
                            loadingManager.RequestRemesh(scene_ptr, obj->get_id());
                        }
                    }
                    redoKeyWasPressed = true;
                }
            } else {
                static bool redoKeyWasPressed = false;
                redoKeyWasPressed = false;
            }
        }

        if (fileDialogResultReady.load()) {
            std::string path;
            bool isImport = isImportDialog;
            {
                std::lock_guard<std::mutex> lock(filePathMutex);
                path = filePathFromDialog;
                filePathFromDialog.clear();
            }
            fileDialogResultReady = false;
            if (!path.empty()) {
                if (isImport) {
                    std::filesystem::path filePath(path);
                    if (filePath.extension() == ".obj") {
                        g_fileToImportPath = path;
                        g_showImportOptionsPopup = true;
                    }
                } else {
                    Urbaxio::FileIO::ExportSceneToObj(path, *scene_ptr);
                }
            }
        }

        // --- Process background job results ---
        Urbaxio::LoadedDataResult loadedData;
        if (loadingManager.PopResult(loadedData)) {
            std::visit([&](auto&& arg) {
                using T = std::decay_t<decltype(arg)>;
                if constexpr (std::is_same_v<T, Urbaxio::FileIO::LoadedSceneData>) {
                    Urbaxio::FileIO::ApplyLoadedDataToScene(arg, *scene_ptr);
                    renderer.InvalidateStaticBatch();
                } else if constexpr (std::is_same_v<T, Urbaxio::VoxelizeResult>) {
                    // Create a VolumetricGeometry from the grid returned by the worker

                    auto new_geom = std::make_unique<Urbaxio::Engine::VolumetricGeometry>(std::move(arg.grid));

                    // Create and execute a command that will swap the geometry

                    // and store the original for undo.
                    auto command = std::make_unique<Urbaxio::Engine::VoxelizeCommand>(

                        scene_ptr,

                        arg.objectId,

                        std::move(new_geom)

                    );
                    scene_ptr->getCommandManager()->ExecuteCommand(std::move(command));
                    
                    // NOTE: First mesh is generated synchronously by VolumetricGeometry::getRenderMesh()
                    // Subsequent updates will be async via RequestRemesh in SculptTool
                    
                    // Deselect after operation

                    selectedObjId = 0;
                    selectedTriangleIndices.clear();
                } else if constexpr (std::is_same_v<T, Urbaxio::RemeshResult>) {
                    // Async remesh completed! Update the mesh instantly
                    std::cout << "[Main] Async remesh complete for object " << arg.objectId << std::endl;
                    
                    Urbaxio::Engine::SceneObject* obj = scene_ptr->get_object_by_id(arg.objectId);
                    if (obj) {
                        // Directly set the mesh buffers (instant swap, no Marching Cubes!)
                        obj->setMeshBuffers(std::move(arg.mesh));
                        renderer.InvalidateStaticBatch();
                    }
                }
            }, loadedData);

        }

        // --- NEW: Check if the scene's static geometry has changed and invalidate renderer if so ---
        if (scene_ptr->IsStaticGeometryDirty()) {
            renderer.InvalidateStaticBatch();
            scene_ptr->ClearStaticGeometryDirtyFlag();
        }

        // --- NEW: Handle file drop after processing events ---
        if (!inputHandler.droppedFilePath.empty()) {
            std::filesystem::path droppedPath(inputHandler.droppedFilePath);
            if (droppedPath.extension() == ".urbx") {
                // It's a project file, load it directly
                for (auto* obj : scene_ptr->get_all_objects()) { if (obj) FreeGPUResources(*obj); }
                LoadProject(inputHandler.droppedFilePath, scene_ptr, camera, renderer);
                RecreateEssentialMarkers(scene_ptr, capsuleRadius, capsuleHeight10m, capsuleHeight5m);
            } else if (droppedPath.extension() == ".obj") {
                g_fileToImportPath = inputHandler.droppedFilePath;
                g_showImportOptionsPopup = true;
            }
            
            inputHandler.droppedFilePath.clear(); // Clear the path to prevent re-triggering
        }

        // --- NEW: Asynchronous Texture Loading ---
        if (scene_ptr && scene_ptr->getMaterialManager()) {
            auto& materials = scene_ptr->getMaterialManager()->GetAllMaterials();
            bool textureWasLoaded = false;
            for (auto& kv : materials) {
                auto& mat = kv.second;
                if (!mat.diffuseTexturePath.empty() && mat.diffuseTextureID == 0) {
                    textureWasLoaded = true;
                    mat.diffuseTextureID = LoadTextureFromFile(mat.diffuseTexturePath);
                    if (mat.diffuseTextureID == 0) {
                        std::cerr << "Shell: Failed to load texture: " << mat.diffuseTexturePath << std::endl;
                        mat.diffuseTexturePath.clear();
                    } else {
                        std::cout << "Shell: Loaded texture '" << mat.diffuseTexturePath << "' with ID " << mat.diffuseTextureID << std::endl;
                    }
                }
            }
            if (textureWasLoaded) {
                renderer.InvalidateStaticBatch();
            }
        }

        // --- Update Active Tool ---
        toolManager.OnUpdate(currentSnap);
        
        // --- GPU Upload for new/modified objects ---
        if (scene_ptr) {
            for (Urbaxio::Engine::SceneObject* obj : scene_ptr->get_all_objects()) {
                if (obj && obj->hasGeometry() && obj->vao == 0) {
                    UploadMeshToGPU(*obj);
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
            // --- ADD THIS BUTTON ---
            ImGui::SameLine();
            if (ImGui::Button("Create Voxel Sphere")) {
                object_counter++;
                std::string name = "VoxelSphere_" + std::to_string(object_counter);

                // 1. Define grid properties (REDUCED RESOLUTION)
                glm::uvec3 dims(32, 32, 32); // <-- CHANGED FROM 64 to 32
                float voxelSize = 0.4f;     // <-- INCREASED to keep size similar
                glm::vec3 origin = glm::vec3(-((float)dims.x * voxelSize) / 2.0f, -((float)dims.y * voxelSize) / 2.0f, 0.0f);
                auto grid = std::make_unique<Urbaxio::Engine::VoxelGrid>(dims, origin, voxelSize);

                // 2. Procedurally generate a sphere SDF
                glm::vec3 sphereCenter = origin + glm::vec3(dims) * voxelSize * 0.5f;
                sphereCenter.z += 1.0f; // Lift it a bit
                float sphereRadius = 4.0f;

                // Use OpenVDB Accessor for efficient batch writes
                Urbaxio::Engine::VoxelGrid::Accessor accessor(*grid);
                for (unsigned int z = 0; z < dims.z; ++z) {
                    for (unsigned int y = 0; y < dims.y; ++y) {
                        for (unsigned int x = 0; x < dims.x; ++x) {
                            glm::vec3 voxelPos = origin + glm::vec3(x, y, z) * voxelSize;
                            float dist = glm::distance(voxelPos, sphereCenter) - sphereRadius;
                            accessor.setValue(x, y, z, dist);
                        }
                    }
                }
                
                // 3. Create SceneObject with VolumetricGeometry
                Urbaxio::Engine::SceneObject* new_obj = scene_ptr->create_object(name);
                if (new_obj) {
                    auto vol_geom = std::make_unique<Urbaxio::Engine::VolumetricGeometry>(std::move(grid));
                    new_obj->setGeometry(std::move(vol_geom));
                }
            }
            // --- ADD THE NEW VOXELIZE BUTTON LOGIC HERE ---
            ImGui::SameLine();
            bool canVoxelize = false;
            if (selectedObjId != 0) {
                Urbaxio::Engine::SceneObject* obj = scene_ptr->get_object_by_id(selectedObjId);
                if (obj && dynamic_cast<Urbaxio::Engine::BRepGeometry*>(obj->getGeometry())) {
                    canVoxelize = true;
                }
            }
            if (!canVoxelize) {
                ImGui::BeginDisabled();
            }
            
            static int voxelize_resolution = 64;
            ImGui::SliderInt("Voxel Resolution", &voxelize_resolution, 32, 512, "%d", ImGuiSliderFlags_Logarithmic);
            ImGui::TextDisabled("Higher = more detail, more memory");
            
            if (ImGui::Button("Voxelize Selected")) {
                if (canVoxelize) {
                    // This is now an async request
                    loadingManager.RequestVoxelize(scene_ptr, selectedObjId, voxelize_resolution);
                }
            }
            if (!canVoxelize) {
                ImGui::EndDisabled();
            }
            
            ImGui::Separator();
            ImGui::Text("File:");
            if (ImGui::Button("New Scene")) {
                // Delete textures from GPU
                if (scene_ptr && scene_ptr->getMaterialManager()) {
                    for (const auto& kv : scene_ptr->getMaterialManager()->GetAllMaterials()) {
                        const auto& mat = kv.second;
                        if (mat.diffuseTextureID != 0) {
                            GLuint id = mat.diffuseTextureID;
                            glDeleteTextures(1, &id);
                        }
                    }
                }
                for (auto* obj : scene_ptr->get_all_objects()) { if (obj) FreeGPUResources(*obj); }
                scene_ptr->NewScene();
                RecreateEssentialMarkers(scene_ptr, capsuleRadius, capsuleHeight10m, capsuleHeight5m);
                renderer.InvalidateStaticBatch(); // --- NEW: Invalidate the static batch
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
                    // Delete old textures before loading new scene
                    if (scene_ptr && scene_ptr->getMaterialManager()) {
                        for (const auto& kv : scene_ptr->getMaterialManager()->GetAllMaterials()) {
                            const auto& mat = kv.second;
                            if (mat.diffuseTextureID != 0) { GLuint id = mat.diffuseTextureID; glDeleteTextures(1, &id); }
                        }
                    }
                    for (auto* obj : scene_ptr->get_all_objects()) { if (obj) FreeGPUResources(*obj); }
                    LoadProject(path, scene_ptr, camera, renderer);
                    RecreateEssentialMarkers(scene_ptr, capsuleRadius, capsuleHeight10m, capsuleHeight5m);
                }
            }

            ImGui::SeparatorText("Exchange");
            if (ImGui::Button("Import")) {
                std::string path = OpenObjDialog(window);
                if (!path.empty()) {
                    g_fileToImportPath = path;
                    g_showImportOptionsPopup = true;
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Export")) {
                std::string path = SaveObjDialog(window);
                if (!path.empty()) {
                    Urbaxio::FileIO::ExportSceneToObj(path, *scene_ptr);
                }
            }

            ImGui::Separator();
            
            if (ImGui::Button("Appearance Settings")) show_style_editor = true;
            ImGui::SameLine(); // <-- NEW
            if (ImGui::Button("Material Editor")) show_material_editor = true; // <-- NEW
            
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
            // --- START OF MODIFICATION ---
            ImGui::SameLine();
            bool isPaint = activeToolType == Urbaxio::Tools::ToolType::Paint;
            if (ImGui::RadioButton("Paint", isPaint)) toolManager.SetTool(Urbaxio::Tools::ToolType::Paint);
            ImGui::SameLine();
            bool isSculpt = activeToolType == Urbaxio::Tools::ToolType::Sculpt;
            if (ImGui::RadioButton("Sculpt", isSculpt)) toolManager.SetTool(Urbaxio::Tools::ToolType::Sculpt);
            
#ifdef URBAXIO_GPU_ENABLED
#if URBAXIO_GPU_ENABLED
            ImGui::SameLine();
            bool isGpuSculpt = activeToolType == Urbaxio::Tools::ToolType::SculptGpu;
            if (ImGui::RadioButton("GPU Sculpt", isGpuSculpt)) toolManager.SetTool(Urbaxio::Tools::ToolType::SculptGpu);
#endif
#endif
            // --- END OF MODIFICATION ---


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
            ImGui::SameLine();
            if (ImGui::Button("CPU Stress Test (1k boxes)")) {
                if (scene_ptr) {
                    for(int i = 0; i < 1000; ++i) {
                        float x = (rand() % 200 - 100) / 10.0f;
                        float y = (rand() % 200 - 100) / 10.0f;
                        float z = (rand() % 100) / 10.0f;
                        scene_ptr->create_box_object("StressBox", 0.1, 0.1, 0.1);
                    }
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
            ImGui::Text("Scene Objects:"); if (scene_ptr) { std::vector<Urbaxio::Engine::SceneObject*> objects = scene_ptr->get_all_objects(); if (objects.empty()) { ImGui::TextDisabled("(No objects yet)"); } else { ImGui::BeginChild("ObjectList", ImVec2(0, 100), true, ImGuiWindowFlags_HorizontalScrollbar); for (const auto* obj : objects) { if (obj) { ImGui::BulletText("%s (ID:%llu)%s%s", obj->get_name().c_str(), obj->get_id(), obj->hasGeometry() ? " [Geo]" : "", (obj->vao != 0) ? " [GPU]" : ""); } } ImGui::EndChild(); } } else { ImGui::TextDisabled("(Scene pointer is null)"); }
            ImGui::End();
        }

        // --- Appearance Settings Window ---
        if (show_style_editor) {
            ImGui::Begin("Appearance Settings", &show_style_editor);
            if (ImGui::CollapsingHeader("Scene Colors")) { ImGui::ColorEdit3("Background", (float*)&clear_color); }
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
                    for (auto* obj : scene_ptr->get_all_objects()) {
                        if (obj && obj->get_name() == "UnitCapsuleMarker10m") {
                            obj->setGeometry(std::make_unique<Urbaxio::Engine::MeshGeometry>(CreateCapsuleMesh(capsuleRadius, capsuleHeight10m)));
                            break;
                        }
                    }
                }
                if (radius_changed || height5m_changed) {
                    for (auto* obj : scene_ptr->get_all_objects()) {
                         if (obj && obj->get_name() == "UnitCapsuleMarker5m") {
                            obj->setGeometry(std::make_unique<Urbaxio::Engine::MeshGeometry>(CreateCapsuleMesh(capsuleRadius, capsuleHeight5m)));
                            break;
                        }
                    }
                }
            }
            ImGui::End();
        }

        // --- NEW: Material Editor Window ---
        if (show_material_editor) {
            ImGui::Begin("Material Editor", &show_material_editor);
            
            Urbaxio::Engine::MaterialManager* matManager = scene_ptr->getMaterialManager();
            if (matManager) {
                // --- Left Pane: Material List ---
                ImGui::BeginChild("MaterialList", ImVec2(150, 0), true);
                static std::string selectedMaterialName = "Default";
                if (ImGui::Button("New Material")) {
                    std::string name = "New_Material";
                    int counter = 1;
                    while(matManager->GetMaterial(name)) {
                        name = "New_Material_" + std::to_string(counter++);
                    }
                    matManager->AddMaterial({name, glm::vec3(0.8f)});
                    selectedMaterialName = name;
                }
                ImGui::Separator();
                // Using a copy of keys because map can be modified
                std::vector<std::string> matNames;
                for (const auto& [name, mat] : matManager->GetAllMaterials()) matNames.push_back(name);
                for (const auto& name : matNames) {
                    if (ImGui::Selectable(name.c_str(), selectedMaterialName == name)) {
                        selectedMaterialName = name;
                    }
                }
                ImGui::EndChild();
                ImGui::SameLine();
                // --- Right Pane: Material Properties ---
                ImGui::BeginGroup();
                Urbaxio::Engine::Material* selectedMat = matManager->GetMaterial(selectedMaterialName);
                if (selectedMat) {
                    ImGui::Text("Editing: %s", selectedMaterialName.c_str());
                    ImGui::Separator();
                    
                    // --- Use Color Picker ---
                    ImVec4 color = ImVec4(selectedMat->diffuseColor.r, selectedMat->diffuseColor.g, selectedMat->diffuseColor.b, 1.0f);
                    if (ImGui::ColorPicker4("##picker", (float*)&color, ImGuiColorEditFlags_NoSidePreview | ImGuiColorEditFlags_NoSmallPreview)) {
                        if (selectedMaterialName != "Default") { // Prevent editing default material color
                           selectedMat->diffuseColor = glm::vec3(color.x, color.y, color.z);
                        }
                    }
                    
                    ImGui::Text("Texture Path: %s", selectedMat->diffuseTexturePath.empty() ? "None" : selectedMat->diffuseTexturePath.c_str());
                    if (ImGui::Button("Assign Texture") && selectedMaterialName != "Default") {
                        // TODO: Open file dialog to select a texture
                    }
                    if (selectedMat->diffuseTextureID != 0) {
                        ImGui::SameLine();
                        if (ImGui::Button("Clear Texture")) {
                            // We don't delete from GPU here, just disconnect. A cleanup task could do it later.
                            selectedMat->diffuseTexturePath.clear();
                            selectedMat->diffuseTextureID = 0;
                        }
                    }
                    // Pass the selected material name to the PaintTool
                    if(auto* paintTool = dynamic_cast<Urbaxio::Tools::PaintTool*>(toolManager.GetTool(Urbaxio::Tools::ToolType::Paint))) {
                        paintTool->SetCurrentMaterial(selectedMaterialName);
                    }
                } else {
                    ImGui::Text("No material selected.");
                }
                ImGui::EndGroup();
            }
            ImGui::End();
        }

        // --- NEW: Render our custom modal popup if it's been triggered ---
        RenderImportOptionsPopup(g_showImportOptionsPopup, g_fileToImportPath, loadingManager);
        if (!vr_mode) {
            RenderLoadingPopup(loadingManager);
        }

        ImGui::Render();

        // -- START OF MODIFICATION: Replace the entire VR render path block --
        if (vr_mode && vrManager->IsInitialized()) {
            // --- VR RENDER PATH ---

            if (vrManager->BeginFrame()) {
                // --- Poll actions and update state (this part is unchanged) ---
                vrManager->PollActions();
                const auto& leftHand = vrManager->GetLeftHandVisual();

                glm::vec3 cyclopsEyePos(0.0f);
                glm::quat cyclopsEyeQuat;
                const auto& vr_views = vrManager->GetViews();
                if (!vr_views.empty()) {
                    glm::mat4 invView1 = glm::inverse(vr_views[0].viewMatrix);
                    cyclopsEyePos = invView1[3];
                    cyclopsEyeQuat = glm::quat_cast(invView1);
                    if (vr_views.size() > 1) {
                        glm::mat4 invView2 = glm::inverse(vr_views[1].viewMatrix);
                        cyclopsEyePos = (cyclopsEyePos + glm::vec3(invView2[3])) * 0.5f;
                        cyclopsEyeQuat = glm::slerp(cyclopsEyeQuat, glm::quat_cast(invView2), 0.5f);
                    }
                }

                bool rightAButtonIsClicked = (vrManager->rightAButtonIsPressed && !rightAButtonWasPressed);
                rightAButtonWasPressed = vrManager->rightAButtonIsPressed;

                // ... (вся логика обновления состояний, контроллеров, UI, инструментов остается здесь)

                // --- NEW: Handle Undo/Redo gesture actions ---
                if (vrManager->triggeredUndoRedoAction == Urbaxio::UndoRedoAction::TriggerUndo) {
                    scene_ptr->getCommandManager()->Undo();
                    vrManager->triggeredUndoRedoAction = Urbaxio::UndoRedoAction::None; // Reset the flag
                    
                    // After Undo: request async remesh for all volumetric objects
                    for (auto* obj : scene_ptr->get_all_objects()) {
                        if (obj && dynamic_cast<Urbaxio::Engine::VolumetricGeometry*>(obj->getGeometry())) {
                            loadingManager.RequestRemesh(scene_ptr, obj->get_id());
                        }
                    }
                } else if (vrManager->triggeredUndoRedoAction == Urbaxio::UndoRedoAction::TriggerRedo) {
                    scene_ptr->getCommandManager()->Redo();
                    vrManager->triggeredUndoRedoAction = Urbaxio::UndoRedoAction::None; // Reset the flag
                    
                    // After Redo: request async remesh for all volumetric objects
                    for (auto* obj : scene_ptr->get_all_objects()) {
                        if (obj && dynamic_cast<Urbaxio::Engine::VolumetricGeometry*>(obj->getGeometry())) {
                            loadingManager.RequestRemesh(scene_ptr, obj->get_id());
                        }
                    }
                }


                shiftDown = vrManager->rightAButtonIsPressed;
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

                // ... (Этот блок кода до `glDisable(GL_FRAMEBUFFER_SRGB)` остается как был)
                
                // --- Disable SRGB for linear rendering space ---
                glDisable(GL_FRAMEBUFFER_SRGB);
                
                // --- REVISED: Prepare override maps for dynamic objects ---
                std::map<uint64_t, glm::mat4> transformOverrides;
                std::map<uint64_t, glm::vec3> colorOverrides;
                std::map<uint64_t, bool> unlitOverrides;
                
                // Get the cumulative world transform from the grab/zoom action
                const glm::mat4& worldTransform = vrManager->GetWorldTransform();

                // --- NEW: Calculate UNSCALED transforms for parenting UI elements ---
                glm::mat4 leftControllerUnscaledTransform(1.0f);
                glm::mat4 rightControllerUnscaledTransform(1.0f);

                // --- NEW: Define all controller colors ---
                const glm::vec3 mainBodyColor(0.11f, 0.11f, 0.11f);
                const glm::vec3 upperDiskColor = mainBodyColor + 0.04f;
                const glm::vec3 middleDiskColor = upperDiskColor + 0.04f;
                const glm::vec3 lowerDiskColor = middleDiskColor + 0.04f;
                
                // --- FIX: Panel inactive color is now same as lower disk ---
                const glm::vec3 panelInactiveColor = lowerDiskColor;
                const glm::vec3 buttonInactiveColor = mainBodyColor;

                const glm::vec3 buttonActivatedColor(0.89f, 0.96f, 0.99f);

                // --- PROCESS LEFT CONTROLLER ---
                if (leftHand.isValid) {
                    glm::mat4 rawPoseMatrix = XrPoseToModelMatrix(leftHand.pose);
                    leftControllerUnscaledTransform = worldTransform * rawPoseMatrix;
                    glm::mat4 localOffset = glm::translate(glm::mat4(1.0f), g_controllerOffsetTranslate * glm::vec3(-1, 1, 1)) * glm::mat4_cast(glm::quat(glm::radians(g_controllerOffsetEuler * glm::vec3(1, -1, -1))));
                    glm::mat4 scaleMatrix = glm::scale(glm::mat4(1.0f), glm::vec3(-g_controllerModelScale, g_controllerModelScale, g_controllerModelScale));
                    glm::mat4 finalPartTransform = leftControllerUnscaledTransform * localOffset * scaleMatrix;
                    
                    for (auto* part : leftController.GetAllParts()) {
                        transformOverrides[part->get_id()] = finalPartTransform;
                        unlitOverrides[part->get_id()] = true;
                    }
                    
                    // Set base colors
                    if (leftController.mainBody)    colorOverrides[leftController.mainBody->get_id()] = mainBodyColor;
                    if (leftController.upperDisk)   colorOverrides[leftController.upperDisk->get_id()] = upperDiskColor;
                    if (leftController.middleDisk)  colorOverrides[leftController.middleDisk->get_id()] = middleDiskColor;
                    if (leftController.lowerDisk)   colorOverrides[leftController.lowerDisk->get_id()] = lowerDiskColor;
                    
                    // Apply interactive colors
                    if (leftController.panel)       colorOverrides[leftController.panel->get_id()] = glm::mix(panelInactiveColor, buttonActivatedColor, vrManager->leftTriggerValue);
                    if (leftController.triggerButton) colorOverrides[leftController.triggerButton->get_id()] = glm::mix(middleDiskColor, buttonActivatedColor, vrManager->leftTriggerValue);
                    if (leftController.grabButton)    colorOverrides[leftController.grabButton->get_id()] = glm::mix(middleDiskColor, buttonActivatedColor, vrManager->leftSqueezeValue);
                    if (leftController.joystick)    colorOverrides[leftController.joystick->get_id()] = glm::mix(buttonInactiveColor, buttonActivatedColor, glm::length(vrManager->leftJoystick));
                    if (leftController.menuButton)  colorOverrides[leftController.menuButton->get_id()] = vrManager->leftMenuButtonIsPressed ? buttonActivatedColor : buttonInactiveColor;
                    if (leftController.buttonAX)    colorOverrides[leftController.buttonAX->get_id()] = vrManager->leftXButtonIsPressed ? buttonActivatedColor : buttonInactiveColor;
                    if (leftController.buttonBY)    colorOverrides[leftController.buttonBY->get_id()] = vrManager->leftYButtonIsPressed ? buttonActivatedColor : buttonInactiveColor;
                    if (vrManager->leftJoystickIsPressed && leftController.joystick) colorOverrides[leftController.joystick->get_id()] = buttonActivatedColor;
                }

                // --- PROCESS RIGHT CONTROLLER ---
                if (rightHand.isValid) {
                    glm::mat4 rawPoseMatrix = XrPoseToModelMatrix(rightHand.pose);
                    rightControllerUnscaledTransform = worldTransform * rawPoseMatrix;
                    glm::mat4 localOffset = glm::translate(glm::mat4(1.0f), g_controllerOffsetTranslate) * glm::mat4_cast(glm::quat(glm::radians(g_controllerOffsetEuler)));
                    glm::mat4 scaleMatrix = glm::scale(glm::mat4(1.0f), glm::vec3(g_controllerModelScale));
                    glm::mat4 finalPartTransform = rightControllerUnscaledTransform * localOffset * scaleMatrix;

                    for (auto* part : rightController.GetAllParts()) {
                        transformOverrides[part->get_id()] = finalPartTransform;
                        unlitOverrides[part->get_id()] = true;
                    }

                    // Set base colors
                    if (rightController.mainBody)    colorOverrides[rightController.mainBody->get_id()] = mainBodyColor;
                    if (rightController.upperDisk)   colorOverrides[rightController.upperDisk->get_id()] = upperDiskColor;
                    if (rightController.middleDisk)  colorOverrides[rightController.middleDisk->get_id()] = middleDiskColor;
                    if (rightController.lowerDisk)   colorOverrides[rightController.lowerDisk->get_id()] = lowerDiskColor;
                    
                    // Apply interactive colors
                    if (rightController.panel)       colorOverrides[rightController.panel->get_id()] = panelInactiveColor;
                    if (rightController.triggerButton) colorOverrides[rightController.triggerButton->get_id()] = glm::mix(middleDiskColor, buttonActivatedColor, vrManager->rightTriggerValue);
                    if (rightController.grabButton)    colorOverrides[rightController.grabButton->get_id()] = glm::mix(middleDiskColor, buttonActivatedColor, vrManager->rightSqueezeValue);
                    if (rightController.joystick)    colorOverrides[rightController.joystick->get_id()] = glm::mix(buttonInactiveColor, buttonActivatedColor, glm::length(vrManager->rightJoystick));
                    if (rightController.menuButton)  colorOverrides[rightController.menuButton->get_id()] = buttonInactiveColor; // No menu button on right
                    if (rightController.buttonAX)    colorOverrides[rightController.buttonAX->get_id()] = vrManager->rightAButtonIsPressed ? buttonActivatedColor : buttonInactiveColor;
                    if (rightController.buttonBY)    colorOverrides[rightController.buttonBY->get_id()] = vrManager->rightBButtonIsPressed ? buttonActivatedColor : buttonInactiveColor;
                    if (vrManager->rightJoystickIsPressed && rightController.joystick) colorOverrides[rightController.joystick->get_id()] = buttonActivatedColor;
                }

                auto* selectTool = (toolManager.GetActiveToolType() == Urbaxio::Tools::ToolType::Select) 
                    ? static_cast<Urbaxio::Tools::SelectTool*>(toolManager.GetActiveTool()) 
                    : nullptr;
                
                bool isJoystickActive = std::abs(vrManager->rightThumbstickY) > 0.1f;
                bool isDraggingBox = selectTool && selectTool->IsVrDragging();

                glm::vec3 rayEnd = vrRayOrigin;
                bool pointerVisible = rightHand.isValid;

                if (selectTool && (isDraggingBox || isJoystickActive || selectTool->IsVrTriggerDown())) {
                    // If we are actively interacting with the SelectTool, truncate the ray at the ghost point.
                    const float VISUAL_DISTANCE = 0.2f;
                    float worldScale = glm::length(glm::vec3(worldTransform[0]));
                    float offset = selectTool->GetVrDragDistanceOffset();
                    float worldDistance = (VISUAL_DISTANCE + offset) * worldScale;
                    
                    rayEnd = vrRayOrigin + vrRayDirection * worldDistance;
                } else {
                    float rayLength = 100.0f;
                    rayEnd = (vrSnap.snapped && vrSnap.type != Urbaxio::SnapType::GRID) 
                           ? vrSnap.worldPoint 
                           : vrRayOrigin + vrRayDirection * rayLength;
                }

                // Left trigger state used for sphere visibility and panel gating
                bool isTriggerPressed = vrManager->leftTriggerValue > 0.5f;

                if (pointerVisible) {
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

                    // Check intersection with the menu sphere widget if it's visible
                    bool isLeftTriggerPressed = vrManager->leftTriggerValue > 0.5f;
                    if (menuSphereWidget && isLeftTriggerPressed) {
                        glm::mat4 sphereWorldTransform = leftControllerUnscaledTransform * menuSphereOffset;
                        glm::vec3 sphereWorldCenter = glm::vec3(sphereWorldTransform[3]);
                        float sphereWorldRadius = (menuSphereDiameter * 0.5f) * glm::length(glm::vec3(sphereWorldTransform[0]));
                        float intersectionDistance;
                        if (glm::intersectRaySphere(vrRayOrigin, vrRayDirection, sphereWorldCenter, sphereWorldRadius * sphereWorldRadius, intersectionDistance)
                            && intersectionDistance > 0.0f
                            && intersectionDistance < glm::distance(vrRayOrigin, rayEnd)) {
                            rayEnd = vrRayOrigin + vrRayDirection * intersectionDistance;
                        }
                    }
                    
                    renderer.UpdateVRPointer(vrRayOrigin, rayEnd, true);
                } else {
                    renderer.UpdateVRPointer({}, {}, false);
                }

                    // --- NEW: Update and Render VR UI Manager ---
                    Urbaxio::UI::Ray worldRay = {vrRayOrigin, vrRayDirection};
                    
                    bool isLeftTriggerPressed = vrManager->leftTriggerValue > 0.5f;
                    
                    // -- START OF MODIFICATION: Pass separate states --
                    vruiManager.Update(worldRay, leftControllerUnscaledTransform, rightControllerUnscaledTransform, rightHand.triggerClicked, rightHand.triggerReleased, rightHand.triggerWasPressed, rightAButtonIsClicked, vrManager->rightAButtonIsPressed, vrManager->rightBButtonIsPressed, vrManager->leftJoystick.y, isLeftTriggerPressed);
                    // -- END OF MODIFICATION --

                    // --- NEW: Menu Sphere WIDGET logic ---
                    bool sphereConsumedClick = false;
                    if (menuSphereWidget) {
                        // Шар-виджет существует и обновляется, только когда зажат левый курок
                        if (isLeftTriggerPressed) {
                            // 1. Рассчитываем мировую трансформацию виджета
                            glm::mat4 sphereWorldTransform = leftControllerUnscaledTransform * menuSphereOffset;
                            
                            // 2. Обновляем состояние ховера
                            Urbaxio::UI::Ray localRay;
                            glm::mat4 invSphereTransform = glm::inverse(sphereWorldTransform);
                            localRay.origin = invSphereTransform * glm::vec4(vrRayOrigin, 1.0f);
                            localRay.direction = glm::normalize(glm::vec3(invSphereTransform * glm::vec4(vrRayDirection, 0.0f)));
                            Urbaxio::UI::HitResult hit = menuSphereWidget->CheckIntersection(localRay);
                            menuSphereWidget->SetHover(hit.didHit);
                            menuSphereWidget->Update(localRay, false, false, false, false, 0.0f);

                            // 3. Логика клика/захвата
                            if ((rightHand.triggerClicked || rightAButtonIsClicked) && hit.didHit) {
                                sphereConsumedClick = true; // Этот клик обработан шаром
                                if (vrManager->rightBButtonIsPressed) {
                                    // B+курок - начинаем захват для перемещения
                                    isGrabbingMenuSphere = true;
                                    grabbedControllerInitialTransform = rightControllerUnscaledTransform;
                                    grabbedInitialSphereWorldTransform = sphereWorldTransform;
                                    
                                    // Логирование информации при начале захвата
                                    glm::vec3 scale, translation, skew;
                                    glm::quat orientation;
                                    glm::vec4 perspective;
                                    glm::decompose(menuSphereOffset, scale, orientation, translation, skew, perspective);
                                    std::cout << "--- Menu Sphere Widget Info ---\n"
                                              << "Offset Translation: (" << translation.x << ", " << translation.y << ", " << translation.z << ")\n"
                                              << "-------------------------------\n";
                                } else {
                                    // Обычный клик - вызываем действие (переключение панели)
                                    menuSphereWidget->HandleClick();
                                }
                            }
                        } else {
                            // Если курок не зажат, сбрасываем состояние ховера
                            menuSphereWidget->SetHover(false);
                        }

                        // 5. Обновляем позицию во время захвата (работает независимо от курка)
                        if (isGrabbingMenuSphere) {
                            glm::mat4 deltaTransform = rightControllerUnscaledTransform * glm::inverse(grabbedControllerInitialTransform);
                            glm::mat4 newWorldTransform = deltaTransform * grabbedInitialSphereWorldTransform;
                            menuSphereOffset = glm::inverse(leftControllerUnscaledTransform) * newWorldTransform;
                        }
                    }
                    // 6. Логика отпускания
                    if (rightHand.triggerReleased) {
                        isGrabbingMenuSphere = false;
                    }
                    
                    // --- НОВЫЙ БЛОК: Проверяем, занят ли UI, и блокируем инструменты ---
                    bool isInteractingWithPanelSystem = vruiManager.IsInteracting();

                    // --- Handle grabbing for panels (grab initialization is now done in VRPanel::Update) ---
                    static bool toolMenuGrabbedLastFrame = false;
                    static bool numpadGrabbedLastFrame = false;
                    
                    if (auto* toolMenuPanel = vruiManager.GetPanel("ToolMenu")) {
                        if (!toolMenuGrabbedLastFrame && toolMenuPanel->isGrabbing && rightHand.isValid) {
                            toolMenuPanel->grabbedControllerInitialTransform = rightControllerUnscaledTransform;
                        }
                        toolMenuGrabbedLastFrame = toolMenuPanel->isGrabbing;
                        
                        if (rightHand.triggerReleased && toolMenuPanel->isGrabbing) {
                            toolMenuPanel->isGrabbing = false;
                            toolMenuPanel->GetOffsetTransform() = glm::inverse(leftControllerUnscaledTransform) * toolMenuPanel->transform;
                        }

                        if(toolMenuPanel->isGrabbing && rightHand.isValid) {
                            glm::mat4 deltaTransform = rightControllerUnscaledTransform * glm::inverse(toolMenuPanel->grabbedControllerInitialTransform);
                            toolMenuPanel->transform = deltaTransform * toolMenuPanel->grabbedInitialTransform;
                        }
                    }

                    if (auto* numpadPanel = vruiManager.GetPanel("NewNumpad")) {
                        if (!numpadGrabbedLastFrame && numpadPanel->isGrabbing && rightHand.isValid) {
                            numpadPanel->grabbedControllerInitialTransform = rightControllerUnscaledTransform;
                        }
                        numpadGrabbedLastFrame = numpadPanel->isGrabbing;
                        
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
                if (toolManager.GetActiveTool() && !isInteractingWithPanelSystem) {
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

                // Update numpad display text
                if (auto* numpadPanel = vruiManager.GetPanel("NewNumpad")) {
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
                    bool clickConsumed = false;
                    if (!sphereConsumedClick) {
                        clickConsumed = vruiManager.HandleClick();
                    }
                    
                    if (!sphereConsumedClick && !clickConsumed && !isInteractingWithPanelSystem) {
                        // If not interacting with UI, it's a world action for the current tool
                        toolManager.OnLeftMouseDown(0, 0, shiftDown, ctrlDown, vrRayOrigin, vrRayDirection);
                    }
                }
                if (rightAButtonIsClicked) {
                    bool clickConsumed = false;
                    if (!sphereConsumedClick) {
                        clickConsumed = vruiManager.HandleClick();
                    }
                    // We don't forward A-button clicks to world tools, only UI.
                }
                if (rightHand.triggerReleased) {
                    // --- ИЗМЕНИ ЭТОТ БЛОК ---
                    if (!isInteractingWithPanelSystem) {
                        auto* selectTool = (toolManager.GetActiveToolType() == Urbaxio::Tools::ToolType::Select) 
                            ? static_cast<Urbaxio::Tools::SelectTool*>(toolManager.GetActiveTool()) 
                            : nullptr;
                        if (selectTool && selectTool->IsVrDragging()) {
                            const auto& vr_views = vrManager->GetViews();
                            if (!vr_views.empty()) {
                                selectTool->FinalizeVrDragSelection(vr_views[0].viewMatrix, shiftDown);
                            }
                        } else {
                            toolManager.OnLeftMouseUp(0, 0, shiftDown, ctrlDown);
                        }
                    }
                }

                // --- MODIFIED: Move line buffer update out of the loop ---
                uint64_t previewObjId = 0;
                if (toolManager.GetActiveToolType() == Urbaxio::Tools::ToolType::Move) {
                    auto* moveTool = static_cast<Urbaxio::Tools::MoveTool*>(toolManager.GetActiveTool());
                    previewObjId = moveTool->GetMovingObjectId();
                }
                renderer.UpdateUserLinesBuffer(scene_ptr->GetAllLines(), *toolContext.selectedLineIDs, previewObjId, scene_ptr);
                toolManager.RenderPreview(renderer, vrSnap);
                
                // --- 1. MULTIVIEW SCENE PASS ---
                if (!vr_views.empty()) {
                    glm::vec3 cyclopsEyePosMultiview(0.0f);
                    glm::vec3 pos1 = glm::inverse(vr_views[0].viewMatrix)[3];
                    glm::vec3 pos2 = (vr_views.size() > 1) ? glm::inverse(vr_views[1].viewMatrix)[3] : pos1;
                    cyclopsEyePosMultiview = (pos1 + pos2) * 0.5f;
                    renderer.setCyclopsEyePosition(cyclopsEyePosMultiview);
                    glm::vec3 cursorWorldPos = vrSnap.snapped ? vrSnap.worldPoint : rayEnd;
                    renderer.RenderFrameMultiview(
                        vrManager->GetMultiviewFbo(),
                        vr_views, 
                        cyclopsEyePosMultiview,
                        scene_ptr,
                        lightColor, ambientStrength, 
                        showGrid, showAxes, axisLineWidth, negAxisLineWidth,
                        gridColor, axisColorX, axisColorY, axisColorZ, positiveAxisFadeColor, negativeAxisFadeColor,
                        cursorWorldPos, cursorRadius, effectIntensity,
                        previewObjId, // --- NEW: Pass preview object ID
                        transformOverrides, colorOverrides, unlitOverrides
                    );
                }
                
                // --- 2. PER-EYE UI & TRANSPARENCY PASS ---
                
                // NEW: Clear the main window framebuffer BEFORE the per-eye loop
                glBindFramebuffer(GL_FRAMEBUFFER, 0);
                glViewport(0, 0, display_w, display_h);
                glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                glm::vec3 cursorWorldPos = vrSnap.snapped ? vrSnap.worldPoint : rayEnd;
                for (uint32_t i = 0; i < vr_views.size(); ++i) {
                    renderer.setCurrentEyeIndex(i);
                    const auto& swapchain = vrManager->GetSwapchain(i);
                    uint32_t imageIndex = vrManager->AcquireSwapchainImage(i);
                    
                    const auto& multiviewFbo = vrManager->GetMultiviewFbo();
                    
                    // Correctly blit from multiview FBO to swapchain FBO for this eye
                    glBindFramebuffer(GL_READ_FRAMEBUFFER, renderer.GetBlitFBO());
                    glFramebufferTextureLayer(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, multiviewFbo.colorTexture, 0, i);
                    glFramebufferTextureLayer(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, multiviewFbo.depthTexture, 0, i);
                    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, swapchain.fbos[imageIndex]);
                    glBlitFramebuffer(0, 0, multiviewFbo.width, multiviewFbo.height,
                                      0, 0, swapchain.width, swapchain.height,
                                      GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT, GL_NEAREST);
                    
                    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
                    // Now bind the swapchain FBO for further drawing of UI elements
                    glBindFramebuffer(GL_FRAMEBUFFER, swapchain.fbos[imageIndex]);
                    glViewport(0, 0, swapchain.width, swapchain.height);
                    const auto& current_view = vr_views[i];
                    const glm::mat4& view = current_view.viewMatrix;
                    const glm::mat4& projection = current_view.projectionMatrix;
                    glm::vec3 viewPos = glm::vec3(glm::inverse(view)[3]);
                    // Render non-multiview elements (highlights, previews, lines, etc.)
                    renderer.RenderFrame(
                        swapchain.width, swapchain.height, view, projection, viewPos, scene_ptr,
                        lightColor, ambientStrength, 
                        false, false, 0, 0, {},{},{},{},{},{}, // Grid/axes already drawn
                        cursorWorldPos, cursorRadius, effectIntensity, 
                        *toolContext.selectedObjId, *toolContext.selectedTriangleIndices, *toolContext.selectedLineIDs, selectionHighlightColor, 
                        *toolContext.hoveredObjId, *toolContext.hoveredFaceTriangleIndices, hoverHighlightColor,
                        vrSnap, nullptr, !selectTool || !selectTool->IsVrDragging(),
                        previewObjId, glm::mat4(1.0f), {}, {}, {}
                    );
                    
                    // Render snap markers and other previews per-eye
                    // ... (This block of code for rendering snap markers and ghost points remains the same)
                    // Re-render snap marker per-eye
                    if (selectTool) {
                        glm::vec3 ghostPoint;
                        selectTool->GetVrGhostPoint(vrRayOrigin, vrRayDirection, ghostPoint);
                        Urbaxio::SnapResult ghostPointSnap = {true, ghostPoint, Urbaxio::SnapType::ENDPOINT};
                        renderer.RenderSnapMarker(ghostPointSnap, view, projection, swapchain.width, swapchain.height);
                    } else {
                        renderer.RenderSnapMarker(vrSnap, view, projection, swapchain.width, swapchain.height);
                    }
                    
                    // Render UI and Text per-eye
                    // ... (The transparentQueue logic for rendering panels, buttons, etc. remains the same)
                    // --- NEW: Back-to-Front Sorted Rendering for Transparent UI (Final Fix) ---
                    std::vector<std::pair<float, std::function<void()>>> transparentQueue;
                    
                    glm::vec3 cameraPos = glm::inverse(view)[3];

                    // 1. Собираем все видимые панели из UI Manager
                    for (const auto& [name, panel] : vruiManager.GetPanels()) {
                        if (panel.alpha > 0.01f) {
                            glm::vec3 panelPos = glm::vec3(panel.transform[3]);
                            float viewSpaceZ = (view * glm::vec4(panelPos, 1.0f)).z;
                            
                            // ПРАВИЛЬНЫЙ ЗАХВАТ: Захватываем указатель 'p' по значению.
                            transparentQueue.push_back({viewSpaceZ, [p = &panel, &renderer, &textRenderer, &view, &projection] {
                                p->Render(renderer, textRenderer, view, projection);
                            }});
                        }
                    }

                    // 2. Собираем виджет-шар
                    // --- START OF MODIFICATION ---
                    // The menu sphere is now rendered independently of any panel, based only on the left trigger press.
                    if (menuSphereWidget && isLeftTriggerPressed) {
                        float sphereAlpha = 1.0f; // It's either visible (1.0) or not rendered at all.

                        if (sphereAlpha > 0.01f) { // This check is a bit redundant now but harmless
                            glm::mat4 sphereWorldTransform = leftControllerUnscaledTransform * menuSphereOffset;
                            glm::vec3 spherePos = glm::vec3(sphereWorldTransform[3]);
                            float viewSpaceZ = (view * glm::vec4(spherePos, 1.0f)).z;
                            
                            transparentQueue.push_back({viewSpaceZ, 
                                [&renderer, &textRenderer, &view, &projection, widget = menuSphereWidget.get(), sphereWorldTransform, sphereAlpha] 
                                {
                                    widget->Render(renderer, textRenderer, sphereWorldTransform, view, projection, sphereAlpha, std::nullopt);
                                }});
                        }
                    }
                    // --- END OF MODIFICATION ---

                    // 3. Сортируем список от дальних к ближним (по возрастанию Z в пространстве камеры)
                    std::sort(transparentQueue.begin(), transparentQueue.end(), [](const auto& a, const auto& b) {
                        return a.first < b.first;
                    });

                    // 4. Отрисовываем все элементы в правильном, отсортированном порядке
                    for (const auto& item : transparentQueue) {
                        item.second(); // Вызываем сохранённую лямбда-функцию рендера
                    }

                    // Complete rewrite of hint text positioning logic
                    // This robust approach treats the multi-line text as a single billboarded panel.
                    
                    // 1. Get the average head pose in appSpace (physical space, Y-up) from raw view data
                    glm::mat4 headPoseInAppSpace;
                    if (vr_views.size() > 1) {
                        glm::mat4 headPose1 = glm::inverse(XrPoseToMat4(vr_views[0].pose));
                        glm::mat4 headPose2 = glm::inverse(XrPoseToMat4(vr_views[1].pose));
                        glm::vec3 posInAppSpace = (glm::vec3(headPose1[3]) + glm::vec3(headPose2[3])) * 0.5f;
                        glm::quat rotInAppSpace = glm::slerp(glm::quat_cast(headPose1), glm::quat_cast(headPose2), 0.5f);
                        headPoseInAppSpace = glm::translate(glm::mat4(1.0f), posInAppSpace) * glm::mat4_cast(rotInAppSpace);
                    } else if (!vr_views.empty()) {
                        headPoseInAppSpace = glm::inverse(XrPoseToMat4(vr_views[0].pose));
                    }
                    // 2. Define text position relative to the head in physical space
                    const float HINT_DISTANCE_FROM_HEAD = 2.0f; // 2 meters in front
                    const float HINT_VERTICAL_OFFSET = -1.2f; // 50 cm below eye level
                    
                    glm::mat4 textAnchorInAppSpace = headPoseInAppSpace 
                                                   * glm::translate(glm::mat4(1.0f), glm::vec3(0, HINT_VERTICAL_OFFSET, -HINT_DISTANCE_FROM_HEAD));
                    // 3. Transform this physical-space anchor point into the virtual world space
                    const glm::mat4& worldTransform = vrManager->GetWorldTransform();
                    glm::mat4 textAnchorInWorldSpace = worldTransform * textAnchorInAppSpace;
                    glm::vec3 hintAnchorPos = glm::vec3(textAnchorInWorldSpace[3]);
                    // 4. Get the correct 'up' vector. It should be the 'up' of the head in the final virtual world,
                    // which prevents the text from rolling with the head.
                    glm::mat4 worldHeadPose = worldTransform * headPoseInAppSpace;
                    glm::vec3 headUp = glm::normalize(glm::vec3(worldHeadPose[1])); // The Y-axis of the transformed head pose matrix

                    float hint_alpha = 0.0f;
                    std::string hint_line1, hint_line2;

                    if (isFileDialogActive.load()) {
                        hint_line1 = "Please check your desktop monitor";
                        hint_line2 = "to select a file.";
                        hint_alpha = 1.0f;
                    } else if (loadingManager.IsLoading()) {
                        hint_line1 = loadingManager.GetStatus();
                        hint_alpha = 1.0f;
                    }

                    if (hint_alpha > 0.01f) {
                        float worldScale = glm::length(glm::vec3(worldTransform[0]));
                        const float desiredPx = 25.0f;
                        float viewportH = static_cast<float>(swapchain.height);
                        const auto& fov = current_view.fov;

                        float viewZ = -(view * glm::vec4(hintAnchorPos, 1.0f)).z;
                        viewZ = std::max(0.1f, viewZ);

                        float worldUnitsPerPixel = viewZ * (tanf(fov.angleUp) - tanf(fov.angleDown)) / viewportH;
                        float textWorldSize = desiredPx * worldUnitsPerPixel;
                        float finalWorldHeight = textWorldSize * worldScale;

                        // 5. Create a single billboard matrix for the entire text block, now using the correct 'up' vector
                        glm::mat4 textBillboardMatrix = glm::inverse(glm::lookAt(hintAnchorPos, cyclopsEyePos, headUp));

                        // 6. Set this transform once and render text relative to it using AddTextOnPanel
                        textRenderer.SetPanelModelMatrix(textBillboardMatrix);
                        
                        const float textOpacity = 0.85f;
                        const glm::vec4 textColor = glm::vec4(1.0f, 1.0f, 1.0f, hint_alpha * textOpacity);
                        
                        if (hint_line2.empty()) {
                            // Render a single centered line
                            textRenderer.AddTextOnPanel(hint_line1, glm::vec3(0.0f), textColor, finalWorldHeight, Urbaxio::TextAlign::CENTER);
                        } else {
                            // Render two lines, offset vertically from the center in the panel's local space
                            float local_y_offset = finalWorldHeight * 0.6f; // Tweak this for line spacing
                            textRenderer.AddTextOnPanel(hint_line1, glm::vec3(0.0f, local_y_offset, 0.0f), textColor, finalWorldHeight, Urbaxio::TextAlign::CENTER);
                            textRenderer.AddTextOnPanel(hint_line2, glm::vec3(0.0f, -local_y_offset, 0.0f), textColor, finalWorldHeight, Urbaxio::TextAlign::CENTER);
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
                    // IMPORTANT: You must copy your existing transparentQueue logic here
                    
                    // This call now clears the text buffers for the NEXT eye/frame
                    textRenderer.Render(view, projection);
                    textRenderer.RenderPanelText(view, projection);
                    textRenderer.ClearPanelModelMatrix();
                    // NEW: Blit to mirror view IF this is the left eye
                    if (i == 0) {
                        glBindFramebuffer(GL_READ_FRAMEBUFFER, swapchain.fbos[imageIndex]);
                        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0); // Window's framebuffer
                        glBlitFramebuffer(
                            0, 0, swapchain.width, swapchain.height,
                            0, 0, display_w, display_h,
                            GL_COLOR_BUFFER_BIT, GL_LINEAR
                        );
                    }
                    glFinish();
                    glBindFramebuffer(GL_FRAMEBUFFER, 0);
                    vrManager->ReleaseSwapchainImage(i);
                }
                vrManager->EndFrame();
            
                // We now draw ImGui on top of the already-blitted mirror view.
                glBindFramebuffer(GL_FRAMEBUFFER, 0);
                glEnable(GL_FRAMEBUFFER_SRGB); // ImGui expects sRGB
                ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
            }
        } 
        // -- END OF MODIFICATION --
        else {
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
                lightColor, ambientStrength, 
                showGrid, showAxes, axisLineWidth, negAxisLineWidth,
                gridColor, axisColorX, axisColorY, axisColorZ, positiveAxisFadeColor, negativeAxisFadeColor,
                cursorWorldPos, cursorRadius, effectIntensity, 
                selectedObjId, selectedTriangleIndices, selectedLineIDs, selectionHighlightColor, 
                hoveredObjId, hoveredFaceTriangleIndices, hoverHighlightColor,
                currentSnap, 
                ImGui::GetDrawData(),
                true, // showSnapMarker - always show for desktop
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
    // --- Texture cleanup ---
    if (scene_ptr && scene_ptr->getMaterialManager()) {
        for (const auto& kv : scene_ptr->getMaterialManager()->GetAllMaterials()) {
            const auto& mat = kv.second;
            if (mat.diffuseTextureID != 0) { GLuint id = mat.diffuseTextureID; glDeleteTextures(1, &id); }
        }
    }
    if (scene_ptr) { for (auto* obj : scene_ptr->get_all_objects()) { if (obj) FreeGPUResources(*obj); } }
    ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplSDL2_Shutdown(); ImGui::DestroyContext(); SDL_GL_DeleteContext(gl_context); SDL_DestroyWindow(window); SDL_Quit(); std::cout << "Shell: Urbaxio Application finished gracefully." << std::endl; return 0;
}