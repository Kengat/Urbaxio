#pragma once

#include "snapping.h"
#include "ui/IVRWidget.h"
#include <engine/line.h> // <--- ADDED: Fix for unknown type 'Line'
#include <engine/Material.h> // <-- ADD THIS
#include <cad_kernel/MeshBuffers.h> // <-- ADDED for Ghost Mesh
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>
#include <string>
#include <SDL2/SDL_video.h>
#include <cstdint>
#include <cstddef>
#include <map>
#include <utility> // For std::pair
#include <set>     // <--- ADDED: Fix for 'std::set' is not a member of 'std'
#include <optional> // <-- ADD THIS

// -- START OF MODIFICATION --
// Forward declare the multiview FBO struct from VRManager
namespace Urbaxio { class VRManager; struct MultiviewFBO; struct VRView; }
// -- END OF MODIFICATION --

namespace Urbaxio { class Camera; namespace Engine { class SceneObject; class Scene; } }
struct ImDrawData;

namespace Urbaxio {

    enum class MarkerShape { CIRCLE, DIAMOND };

    // --- NEW: Struct for a single draw call ---
    // --- MODIFIED: This struct now holds offsets for batching instead of a VAO ---
    struct RenderCommand {
        const Engine::Material* material;
        // GLuint vao; // No longer needed, we use one static VAO for all batched objects
        GLsizei indexCount;
        size_t startIndexOffset; // Byte offset into the combined static IBO
        int baseVertex;          // Vertex offset into the combined static VBOs
        glm::mat4 modelMatrix;
        bool isUnlit;
        std::optional<glm::vec3> colorOverride;
        uint64_t objectId = 0; // NEW: ID of the source object for this command
    };

    class Renderer {
    public:
        Renderer();
        ~Renderer();
    bool Initialize();
        void RenderFrame(
            int viewportWidth, int viewportHeight,
            const glm::mat4& view, const glm::mat4& projection, const glm::vec3& viewPos,
        Urbaxio::Engine::Scene* scene,
        // Appearance
        const glm::vec3& lightColor, float ambientStrength,
        bool showGrid, bool showAxes, float axisLineWidth, float negAxisLineWidth,
        const glm::vec3& gridColor, const glm::vec4& axisColorX, const glm::vec4& axisColorY, const glm::vec4& axisColorZ,
        const glm::vec4& positiveAxisFadeColor, const glm::vec4& negativeAxisFadeColor,
        // Interactive Effects
        const glm::vec3& cursorWorldPos, float cursorRadius, float intensity,
        // Selections
        uint64_t selectedObjId,
        const std::vector<size_t>& selectedTriangleIndices,
        const std::set<uint64_t>& selectedLineIDs,
        const glm::vec3& selectionHighlightColor,
        // Hovers
        uint64_t hoveredObjId,
        const std::vector<size_t>& hoveredFaceTriangleIndices,
        const glm::vec3& hoverHighlightColor,
        // Tools
        const SnapResult& currentSnap,
        ImDrawData* imguiDrawData,
        bool showSnapMarker = true, // NEW: Control snap marker visibility
        // --- NEW for Previews ---
        uint64_t previewObjectId = 0,
        const glm::mat4& previewTransform = glm::mat4(1.0f),
        // --- NEW: Overrides for dynamic objects like controllers ---
        const std::map<uint64_t, glm::mat4>& transformOverrides = {},
        const std::map<uint64_t, glm::vec3>& colorOverrides = {},
        const std::map<uint64_t, bool>& unlitOverrides = {}
    );
        // -- START OF MODIFICATION --
        void RenderFrameMultiview(
            const MultiviewFBO& multiviewFbo,
            const std::vector<VRView>& views, const glm::vec3& viewPos,
            Urbaxio::Engine::Scene* scene,
            // Appearance
            const glm::vec3& lightColor, float ambientStrength,
            bool showGrid, bool showAxes, float axisLineWidth, float negAxisLineWidth,
            const glm::vec3& gridColor, const glm::vec4& axisColorX, const glm::vec4& axisColorY, const glm::vec4& axisColorZ,
            const glm::vec4& positiveAxisFadeColor, const glm::vec4& negativeAxisFadeColor,
            // Interactive Effects
            const glm::vec3& cursorWorldPos, float cursorRadius, float intensity,
            // --- NEW for Previews ---
            uint64_t previewObjectId = 0,
            // Overrides
            const std::map<uint64_t, glm::mat4>& transformOverrides = {},
            const std::map<uint64_t, glm::vec3>& colorOverrides = {},
            const std::map<uint64_t, bool>& unlitOverrides = {}
        );
        // -- END OF MODIFICATION --
        void SetViewport(int x, int y, int width, int height);
        float GetMaxLineWidth() const { return maxLineWidth; }

        // Buffer updaters for tools and scene state
        void UpdateUserLinesBuffer(
            const std::map<uint64_t, Engine::Line>& lines, 
            const std::set<uint64_t>& selectedLineIDs, 
            uint64_t movingObjectId, 
            Engine::Scene* scene
        );
        void UpdatePreviewLine(const glm::vec3& start, const glm::vec3& end, bool enabled = true);
        void UpdateVRPointer(const glm::vec3& start, const glm::vec3& end, bool enabled); // <-- NEW
        void UpdatePushPullPreview(const Engine::SceneObject& object, const std::vector<size_t>& faceIndices, const glm::vec3& direction, float distance);
        void UpdateAxesVBO(const glm::vec4& colorX, const glm::vec4& colorY, const glm::vec4& colorZ, const glm::vec4& posFadeColor, const glm::vec4& negFadeColor);
        void RenderSelectionBox(const glm::vec2& start, const glm::vec2& end, int screenWidth, int screenHeight); // <-- NEW

        // NEW: Preview Box for 3D selection
        void UpdatePreviewBox(const glm::vec3& p1, const glm::vec3& p2, bool enabled);
        void UpdateDragStartPoint(const glm::vec3& point, bool enabled);
        void UpdateBrushPreview(const glm::vec3& position, float radius, const glm::vec3& color, bool enabled);
        
        void RenderVRMenuWidget(
            const glm::mat4& view, const glm::mat4& projection,
            const glm::mat4& model,
            const glm::vec3& baseColor, float aberration, float globalAlpha,
            const glm::vec3& aberrationColor1, const glm::vec3& aberrationColor2,
            GLuint textureId = 0,
            const std::optional<UI::MaskData>& mask = std::nullopt
        );

        // --- NEW: Render a simple textured quad for desktop mirror ---
        void RenderVRTexture(
            const glm::mat4& view, const glm::mat4& projection,
            const glm::mat4& model,
            GLuint textureId,
            float alpha,
            const std::optional<UI::MaskData>& mask = std::nullopt
        );

        // --- MODIFIED: Added mask support ---
        void RenderVRPanel(
            const glm::mat4& view, const glm::mat4& projection,
            const glm::mat4& model,
            const glm::vec3& color, float cornerRadius, float alpha,
            const std::optional<UI::MaskData>& mask = std::nullopt
        );
        // ------------------------------------

        // --- NEW: Ghost Mesh methods ---
        void UpdateGhostMesh(const CadKernel::MeshBuffers& mesh, const std::vector<unsigned int>& wireframeIndices);
        void ClearGhostMesh();

        // Render a single object with custom transform and color (for VR controllers)
        void RenderSingleObject(
            const glm::mat4& view, const glm::mat4& projection,
            Urbaxio::Engine::SceneObject* obj,
            const glm::mat4& modelMatrix,
            const glm::vec3& color,
            bool unlit = false
        );

        // Public wrapper to allow tools to draw snap-style markers
        void RenderSnapMarker(const SnapResult& snap, const glm::mat4& view, const glm::mat4& proj, int viewportWidth, int viewportHeight);

        // --- NEW: Public method to invalidate the static geometry batch ---
        void InvalidateStaticBatch();

        void setCyclopsEyePosition(const glm::vec3& pos);
        glm::vec3 getCyclopsEyePosition() const;

        // ДОБАВЬ ЭТИ ДВЕ СТРОКИ

        void setCurrentEyeIndex(int index);

        int getCurrentEyeIndex() const;

        // ДОБАВЬТЕ ЭТОТ МЕТОД:
        void GetViewport(glm::vec4& outViewport) const;

        GLuint GetBlitFBO() const { return blitFBO_; }

        GLuint vrMenuWidgetShaderProgram = 0; // For menu spheres
        glm::mat4 vrMenuWidgetShaderProgram_viewMatrix_HACK = glm::mat4(1.0f);

    private:
        // --- NEW: Structs to cache uniform locations ---
        struct ObjectShaderLocations {
            GLuint model, view, projection, lightDir, lightColor, ambientStrength;
            GLuint objectColor, useTexture, diffuseTexture, overrideAlpha, unlit;
        } objectShaderLocs;

        struct SimpleLineShaderLocations {
            GLuint model, view, projection;
        } simpleLineShaderLocs;

        struct GridShaderLocations {
            GLuint model, view, projection, gridColor, cursorWorldPos, cursorRadius, intensity, holeStart, holeEnd;
        } gridShaderLocs;
        
        struct AxisShaderLocations {
            GLuint model, view, projection, cursorWorldPos, cursorRadius, intensity;
            GLuint fadeStart, fadeEnd, holeStart, holeEnd, positiveFadeColor, negativeFadeColor;
        } axisShaderLocs;

        // -- START OF MODIFICATION --
        struct ObjectMultiviewShaderLocations {
            GLuint model, view, projection, lightDir, lightColor, ambientStrength;
            GLuint objectColor, useTexture, diffuseTexture, overrideAlpha, unlit;
        } objectMultiviewShaderLocs;
        struct GridMultiviewShaderLocations {
            GLuint model, view, projection, gridColor, cursorWorldPos, cursorRadius, intensity, holeStart, holeEnd;
        } gridMultiviewShaderLocs;
        
        struct AxisMultiviewShaderLocations {
            GLuint model, view, projection, cursorWorldPos, cursorRadius, intensity;
            GLuint fadeStart, fadeEnd, holeStart, holeEnd, positiveFadeColor, negativeAxisFadeColor;
        } axisMultiviewShaderLocs;
        // -- END OF MODIFICATION --

        struct DashedLineShaderLocations {
            GLuint model, view, projection, color, dashSize, gapSize;
        } dashedLineShaderLocs;
        struct BubbleShaderLocations {
            GLuint model, view, projection;
            GLuint color, viewPos;
        } bubbleShaderLocs;

        GLuint objectShaderProgram = 0;
        // --- NEW SHADERS ---
        GLuint gridShaderProgram = 0;
        GLuint axisShaderProgram = 0;
        // -- START OF MODIFICATION --
        GLuint objectMultiviewShaderProgram = 0;
        GLuint gridMultiviewShaderProgram = 0;
        GLuint axisMultiviewShaderProgram = 0;
        // -- END OF MODIFICATION --
        // --- OLD SHADER (renamed for clarity, used for user lines and rubber band) ---
        GLuint simpleLineShaderProgram = 0; 
        GLuint unlitShaderProgram = 0; // <-- NEW SHADER for unlit objects
        GLuint dashedLineShaderProgram = 0; // <-- NEW SHADER for dashed lines
        GLuint selectionBoxShaderProgram = 0; // <-- NEW
        GLuint bubbleShaderProgram = 0;
        
        GLuint splatShaderProgram = 0;
        GLuint markerShaderProgram = 0;

        // --- NEW: Buffers for batched highlighting ---
        std::vector<GLsizei> highlightCounts;
        std::vector<const void*> highlightIndices;

        GLuint gridVAO = 0, gridVBO = 0; int gridVertexCount = 0;
        
        // --- MODIFIED for AXES ---
        GLuint axesVAO = 0, axesVBO = 0;
        int axisVertexCountX = 0;
        int axisVertexCountY = 0;
        int axisVertexCountZ = 0;

        GLuint splatVAO = 0, splatVBO = 0, splatEBO = 0;
        GLuint userLinesVAO = 0, userLinesVBO = 0; int userLinesVertexCount = 0;
        GLuint movingLinesVAO = 0, movingLinesVBO = 0; int movingLinesVertexCount = 0; // <-- NEW for moving lines preview
        
        // --- Preview Resources ---
        GLuint previewVAO = 0, previewVBO = 0; GLsizei previewVertexCount = 0;
        GLuint previewLineVAO = 0, previewLineVBO = 0; bool previewLineEnabled = false;
        GLuint previewOutlineVAO = 0, previewOutlineVBO = 0; GLsizei previewOutlineVertexCount = 0; // <-- NEW for dashed outline
        GLuint selectionBoxVAO = 0, selectionBoxVBO = 0; // <-- NEW

        // NEW: Preview Box Resources
        GLuint previewBoxVAO = 0, previewBoxVBO = 0;
        GLuint previewBoxEBO_triangles = 0, previewBoxEBO_lines = 0;
        bool previewBoxEnabled = false;
        GLsizei previewBoxTriangleIndexCount = 0;
        GLsizei previewBoxLineIndexCount = 0;

        // --- ADD THESE TWO LINES ---
        GLuint panelOutlineVAO_ = 0;
        GLuint panelOutlineVBO_ = 0;

        // NEW: Drag start point marker
        glm::vec3 dragStartPoint;
        bool dragStartPointEnabled = false;

        // --- NEW: VR Pointer Resources ---
        GLuint vrPointerVAO = 0, vrPointerVBO = 0;
        bool vrPointerEnabled = false;

        // --- NEW: Ghost Mesh Resources ---
        GLuint ghostMeshVAO = 0;
        GLuint ghostMeshVBO_vertices = 0;
        GLuint ghostMeshVBO_normals = 0;
        GLuint ghostMeshEBO_triangles = 0; // <-- ПЕРЕИМЕНОВАТЬ
        GLuint ghostMeshEBO_lines = 0;     // <-- ДОБАВИТЬ
        GLsizei ghostMeshTriangleIndexCount = 0; // <-- ПЕРЕИМЕНОВАТЬ
        GLsizei ghostMeshLineIndexCount = 0;     // <-- ДОБАВИТЬ
        GLuint sphereVAO = 0, sphereVBO = 0, sphereEBO = 0;
        GLsizei sphereIndexCount = 0;

        // Unit Quad Resources for desktop mirror texture widgets
        GLuint unitQuadVAO = 0; 
        GLuint unitQuadVBO = 0;

        glm::vec3 brushPreviewPos_{0,0,0};
        float brushPreviewRadius_ = 1.0f;
        glm::vec3 brushPreviewColor_{0.2f, 0.8f, 1.0f};
        bool brushPreviewEnabled_ = false;

        std::map<MarkerShape, GLuint> markerVAOs;
        std::map<MarkerShape, GLuint> markerVBOs;
        std::map<MarkerShape, int> markerVertexCounts;

        float markerScreenSize = 12.0f;
        float markerScreenSizeMidpoint = 10.0f;
        float markerScreenSizeOnEdge = 10.0f;
        glm::vec4 snapMarkerColorPoint = glm::vec4(1.0f, 0.6f, 0.0f, 0.9f);
        glm::vec4 snapMarkerColorMidpoint = glm::vec4(0.5f, 0.8f, 1.0f, 0.9f);
        glm::vec4 snapMarkerColorOnEdge = glm::vec4(1.0f, 0.0f, 1.0f, 0.9f);
        glm::vec4 snapMarkerColorOnFace = glm::vec4(0.2f, 1.0f, 0.8f, 0.7f);
        glm::vec4 snapMarkerColorAxisX = glm::vec4(1.0f, 0.3f, 0.3f, 0.9f);
        glm::vec4 snapMarkerColorAxisY = glm::vec4(0.3f, 1.0f, 0.3f, 0.9f);
        glm::vec4 snapMarkerColorAxisZ = glm::vec4(0.4f, 0.4f, 1.0f, 0.9f);

        glm::vec3 gridColor = glm::vec3(0.4f, 0.5f, 0.6f);
        float gridSizeF = 500.0f; int gridSteps = 500; int gridAccentStep = 10;
        float axisLength = 1000.0f;
        glm::vec3 splatPosStatic = glm::vec3(5.0f, 5.0f, 1.0f);
        glm::vec3 splatPosBillboard = glm::vec3(-5.0f, 5.0f, 1.0f);
        glm::vec4 splatColor = glm::vec4(0.8f, 0.4f, 0.8f, 0.5f);
        float splatBlurStrength = 8.0f;
        float maxLineWidth = 1.0f;
        glm::vec4 userLineColor = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);
        glm::vec4 selectedUserLineColor = glm::vec4(1.0f, 0.65f, 0.0f, 1.0f);
        glm::vec3 cyclopsEyePosition_{0.0f, 0.0f, 0.0f};
        // ДОБАВЬ ЭТУ СТРОКУ

        int currentEyeIndex_ = 0;

        bool CreateShaderPrograms();
        // -- START OF MODIFICATION --
        bool CreateMultiviewShaderPrograms();
        // -- END OF MODIFICATION --
        bool CreateGridResources();
        bool CreateAxesResources();
        bool CreateSplatResources();
        bool CreateUserLinesResources();
        bool CreateMarkerResources();
        bool CreatePreviewResources();
        bool CreatePreviewLineResources();
        bool CreatePreviewOutlineResources();
        bool CreateSelectionBoxResources();
        bool CreatePreviewBoxResources(); // <-- NEW
        bool CreateVRPointerResources(); // <-- NEW
        bool CreateGhostMeshResources(); // <-- NEW
        bool CreatePanelOutlineResources(); // <-- NEW
        bool CreateBubbleResources();
        bool CreateUnitQuadResources(); // Dedicated resource for texture widgets
        void Cleanup();
        void DrawSnapMarker(const SnapResult& snap, const glm::mat4& view, const glm::mat4& proj, int viewportWidth, int viewportHeight);
        
        // --- NEW: "Scene Compiler" for Static Batching ---
        void CompileStaticScene(Urbaxio::Engine::Scene* scene);
        bool isStaticBatchValid_ = false;
        GLuint VAO_Static = 0;
        GLuint VBO_Static_Vertices = 0;
        GLuint VBO_Static_Normals = 0;
        GLuint VBO_Static_UVs = 0;
        GLuint IBO_Static = 0;
        // The queue of commands generated by the scene compiler
        std::map<const Engine::Material*, std::vector<RenderCommand>> staticBatchQueue;
        void DrawPointMarker(const glm::vec3& worldPoint, const glm::vec4& color, const glm::mat4& view, const glm::mat4& proj, int viewportWidth, int viewportHeight);

        const char* objectVertexShaderSource; const char* objectFragmentShaderSource;
        const char* simpleLineVertexShaderSource; const char* simpleLineFragmentShaderSource;
        const char* gridVertexShaderSource; const char* gridFragmentShaderSource;
        const char* axisVertexShaderSource; const char* axisFragmentShaderSource;
        const char* unlitVertexShaderSource; const char* unlitFragmentShaderSource;
        const char* splatVertexShaderSource; const char* splatFragmentShaderSource;
        const char* markerVertexShaderSource; const char* markerFragmentShaderSource;
        const char* dashedLineVertexShaderSource; const char* dashedLineFragmentShaderSource;
        const char* selectionBoxVertexShaderSource; const char* selectionBoxFragmentShaderSource;
        const char* vrMenuWidgetVertexShaderSource; const char* vrMenuWidgetFragmentShaderSource;
        const char* bubbleVertexShaderSource; const char* bubbleFragmentShaderSource;

        // -- START OF MODIFICATION --
        GLuint blitFBO_ = 0; // Temporary FBO for multiview blitting
        // -- END OF MODIFICATION --
    };
}