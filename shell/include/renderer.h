#pragma once

#include "snapping.h"
#include <engine/line.h> // <--- ADDED: Fix for unknown type 'Line'
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

namespace Urbaxio { class Camera; namespace Engine { class SceneObject; class Scene; } }
struct ImDrawData;

namespace Urbaxio {

    enum class MarkerShape { CIRCLE, DIAMOND };

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
        const glm::vec3& defaultObjectColor,
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
        // --- NEW for Previews ---
        uint64_t previewObjectId = 0,
        const glm::mat4& previewTransform = glm::mat4(1.0f),
        // --- NEW: Overrides for dynamic objects like controllers ---
        const std::map<uint64_t, glm::mat4>& transformOverrides = {},
        const std::map<uint64_t, glm::vec3>& colorOverrides = {},
        const std::map<uint64_t, bool>& unlitOverrides = {}
    );
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

    private:
        GLuint objectShaderProgram = 0;
        // --- NEW SHADERS ---
        GLuint gridShaderProgram = 0;
        GLuint axisShaderProgram = 0;
        // --- OLD SHADER (renamed for clarity, used for user lines and rubber band) ---
        GLuint simpleLineShaderProgram = 0; 
        GLuint unlitShaderProgram = 0; // <-- NEW SHADER for unlit objects
        GLuint dashedLineShaderProgram = 0; // <-- NEW SHADER for dashed lines
        GLuint selectionBoxShaderProgram = 0; // <-- NEW
        
        GLuint splatShaderProgram = 0;
        GLuint markerShaderProgram = 0;

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

        bool CreateShaderPrograms();
        bool CreateGridResources();
        bool CreateAxesResources();
        bool CreateSplatResources();
        bool CreateUserLinesResources();
        bool CreateMarkerResources();
        bool CreatePreviewResources();
        bool CreatePreviewLineResources();
        bool CreatePreviewOutlineResources();
        bool CreateSelectionBoxResources();
        bool CreateVRPointerResources(); // <-- NEW
        bool CreateGhostMeshResources(); // <-- NEW
        void Cleanup();
        void DrawSnapMarker(const SnapResult& snap, const glm::mat4& view, const glm::mat4& proj, int viewportWidth, int viewportHeight);

        const char* objectVertexShaderSource; const char* objectFragmentShaderSource;
        const char* simpleLineVertexShaderSource; const char* simpleLineFragmentShaderSource;
        const char* gridVertexShaderSource; const char* gridFragmentShaderSource;
        const char* axisVertexShaderSource; const char* axisFragmentShaderSource;
        const char* unlitVertexShaderSource; const char* unlitFragmentShaderSource;
        const char* splatVertexShaderSource; const char* splatFragmentShaderSource;
        const char* markerVertexShaderSource; const char* markerFragmentShaderSource;
        const char* dashedLineVertexShaderSource; const char* dashedLineFragmentShaderSource;
        const char* selectionBoxVertexShaderSource; const char* selectionBoxFragmentShaderSource;
    };
}