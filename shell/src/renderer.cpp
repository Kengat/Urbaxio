#include "renderer.h"
#include "camera.h"
#include <engine/scene.h>
#include <engine/scene_object.h>
#include <engine/line.h>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <imgui_impl_opengl3.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstddef>
#include <map>
#include <set>
#include <utility>

namespace { // Anonymous namespace for utility functions
    GLuint CompileShader(GLenum type, const char* source) { GLuint shader = glCreateShader(type); glShaderSource(shader, 1, &source, NULL); glCompileShader(shader); GLint success; GLchar infoLog[512]; glGetShaderiv(shader, GL_COMPILE_STATUS, &success); if (!success) { glGetShaderInfoLog(shader, 512, NULL, infoLog); std::cerr << "ERROR::SHADER::COMPILATION_FAILED\n" << (type == GL_VERTEX_SHADER ? "Vertex" : "Fragment") << "\n" << source << "\n" << infoLog << std::endl; glDeleteShader(shader); return 0; } return shader; }
    GLuint LinkShaderProgram(GLuint vertexShader, GLuint fragmentShader) { GLuint shaderProgram = glCreateProgram(); glAttachShader(shaderProgram, vertexShader); glAttachShader(shaderProgram, fragmentShader); glLinkProgram(shaderProgram); GLint success; GLchar infoLog[512]; glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success); if (!success) { glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog); std::cerr << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl; glDeleteProgram(shaderProgram); if (glIsShader(vertexShader)) glDeleteShader(vertexShader); if (glIsShader(fragmentShader)) glDeleteShader(fragmentShader); return 0; } glDeleteShader(vertexShader); glDeleteShader(fragmentShader); return shaderProgram; }
    // Modified to only generate geometry, color will be handled by shader
    // Generates grid vertices on the XY plane (for Z-up system)
    std::vector<float> GenerateGridVertices(float size, int steps) {
        std::vector<float> vertices;
        if (steps <= 0 || size <= 0.0f) return vertices;
        float halfSize = size / 2.0f;
        float stepSize = size / static_cast<float>(steps);
        const float epsilon = 1e-6f;

        for (int i = 0; i <= steps; ++i) {
            float pos = -halfSize + static_cast<float>(i) * stepSize;
            
            // Skip center lines, as they are drawn by the axes
            if (std::abs(pos) > epsilon) {
                // Lines parallel to Y axis (X is constant at 'pos')
                vertices.push_back(pos); vertices.push_back(-halfSize); vertices.push_back(0.0f);
                vertices.push_back(pos); vertices.push_back(halfSize);  vertices.push_back(0.0f);

                // Lines parallel to X axis (Y is constant at 'pos')
                vertices.push_back(-halfSize); vertices.push_back(pos); vertices.push_back(0.0f);
                vertices.push_back(halfSize);  vertices.push_back(pos); vertices.push_back(0.0f);
            }
        }
        return vertices;
    }
    // Modified to return vertices for a single axis with specific fade colors
    std::vector<float> GenerateSingleAxisVertices(float length, const glm::vec3& direction, const glm::vec4& baseColor, const glm::vec4& posFadeColor, const glm::vec4& negFadeColor) {
        std::vector<float> vertices;
        if (length <= 0.0f) return vertices;

        glm::vec3 p1 = direction * length;
        glm::vec3 p2 = direction * -length;
        
        // Positive part - use normal alpha 
        vertices.push_back(0.0f); vertices.push_back(0.0f); vertices.push_back(0.0f);
        vertices.push_back(baseColor.r); vertices.push_back(baseColor.g); vertices.push_back(baseColor.b); vertices.push_back(baseColor.a);
        vertices.push_back(p1.x); vertices.push_back(p1.y); vertices.push_back(p1.z);
        vertices.push_back(baseColor.r); vertices.push_back(baseColor.g); vertices.push_back(baseColor.b); vertices.push_back(baseColor.a);

        // Negative part - use negative alpha to mark it for the shader
        vertices.push_back(0.0f); vertices.push_back(0.0f); vertices.push_back(0.0f);
        vertices.push_back(baseColor.r); vertices.push_back(baseColor.g); vertices.push_back(baseColor.b); vertices.push_back(-negFadeColor.a);
        vertices.push_back(p2.x); vertices.push_back(p2.y); vertices.push_back(p2.z);
        vertices.push_back(baseColor.r); vertices.push_back(baseColor.g); vertices.push_back(baseColor.b); vertices.push_back(-negFadeColor.a);

        return vertices;
    }
    std::vector<float> GenerateQuadVertices(float size) { float half = size / 2.0f; std::vector<float> vertices = { -half,-half,0.0f, 0.0f,0.0f, half,-half,0.0f, 1.0f,0.0f, half, half,0.0f, 1.0f,1.0f, -half, half,0.0f, 0.0f,1.0f }; return vertices; }
    std::vector<float> GenerateCircleVertices(int segments) { std::vector<float> vertices; vertices.push_back(0.0f); vertices.push_back(0.0f); float angleStep = 2.0f * glm::pi<float>() / static_cast<float>(segments); for (int i = 0; i <= segments; ++i) { float angle = static_cast<float>(i) * angleStep; vertices.push_back(cos(angle) * 0.5f); vertices.push_back(sin(angle) * 0.5f); } return vertices; }
    std::vector<float> GenerateDiamondVertices() { std::vector<float> vertices = { 0.0f,  0.5f, 0.5f,  0.0f, 0.0f, -0.5f, -0.5f,  0.0f }; vertices.push_back(0.0f); vertices.push_back(0.5f); return vertices; }
}

namespace Urbaxio {

    Renderer::Renderer() {
        objectVertexShaderSource =
            "#version 330 core\n"
            "layout (location = 0) in vec3 aPos;\n"
            "layout (location = 1) in vec3 aNormal;\n"
            "uniform mat4 model;\n"
            "uniform mat4 view;\n"
            "uniform mat4 projection;\n"
            "out vec3 FragPosWorld;\n"
            "out vec3 NormalWorld;\n"
            "void main() {\n"
            "    FragPosWorld = vec3(model * vec4(aPos, 1.0));\n"
            "    NormalWorld = mat3(transpose(inverse(model))) * aNormal;\n"
            "    gl_Position = projection * view * vec4(FragPosWorld, 1.0);\n"
            "}\n";

        objectFragmentShaderSource =
            "#version 330 core\n"
            "out vec4 FragColor;\n"
            "in vec3 FragPosWorld;\n"
            "in vec3 NormalWorld;\n"
            "uniform vec3 objectColor;\n"
            "uniform vec3 lightDir;\n"
            "uniform vec3 lightColor;\n"
            "uniform float ambientStrength;\n"
            "uniform vec3 viewPos;\n"
            "uniform float overrideAlpha = 1.0;\n"
            "uniform bool u_unlit = false;\n" // <-- ДОБАВИТЬ uniform
            "void main() {\n"
            "    vec3 result;\n"
            "    if (u_unlit) {\n" // <-- ДОБАВИТЬ ЭТОТ БЛОК
            "        result = objectColor;\n"
            "    } else {\n"
            "        vec3 norm = normalize(NormalWorld);\n"
            "        vec3 ambient = ambientStrength * lightColor * objectColor;\n"
            "        float diff = max(dot(norm, normalize(lightDir)), 0.0);\n"
            "        vec3 diffuse = diff * lightColor * objectColor;\n"
            "        result = ambient + diffuse;\n"
            "    }\n"
            "    FragColor = vec4(result, overrideAlpha);\n"
            "}\n";

        // This will be for simple lines like the rubber band and user lines
        simpleLineVertexShaderSource =
            "#version 330 core\n"
            "layout (location = 0) in vec3 aPos;\n"
            "layout (location = 1) in vec4 aBaseColorAlpha;\n"
            "uniform mat4 model;\n"
            "uniform mat4 view;\n"
            "uniform mat4 projection;\n"
            "out vec4 fragmentColor;\n"
            "void main() {\n"
            "    gl_Position = projection * view * model * vec4(aPos, 1.0);\n"
            "    fragmentColor = aBaseColorAlpha;\n"
            "}\n";

        simpleLineFragmentShaderSource =
            "#version 330 core\n"
            "out vec4 FragColor;\n"
            "in vec4 fragmentColor;\n"
            "void main() {\n"
            "    FragColor = fragmentColor;\n"
            "}\n";

        gridVertexShaderSource =
            "#version 330 core\n"
            "layout (location = 0) in vec3 aPos;\n"
            "uniform mat4 model;\n"
            "uniform mat4 view;\n"
            "uniform mat4 projection;\n"
            "out vec3 vWorldPos;\n"
            "void main() {\n"
            "    vWorldPos = (model * vec4(aPos, 1.0)).xyz;\n"
            "    gl_Position = projection * view * vec4(vWorldPos, 1.0);\n"
            "}\n";

        gridFragmentShaderSource =
            "#version 330 core\n"
            "out vec4 FragColor;\n"
            "in vec3 vWorldPos;\n"
            "uniform vec3 u_gridColor;\n"
            "uniform vec3 u_cursorWorldPos;\n"
            "uniform float u_cursorRadius;\n"
            "uniform float u_intensity;\n"
            "uniform float u_holeStart;\n"
            "uniform float u_holeEnd;\n"
            "const float GRID_FADE_START = 10.0;\n"
            "const float GRID_FADE_END = 40.0;\n"
            "const float BASE_GRID_ALPHA = 0.3;\n"
            "void main() {\n"
            "    float distFromOrigin = length(vWorldPos.xy);\n"
            "    float alphaFromOrigin = (1.0 - smoothstep(GRID_FADE_START, GRID_FADE_END, distFromOrigin)) * BASE_GRID_ALPHA;\n"
            "    float distFromMouse = distance(vWorldPos.xy, u_cursorWorldPos.xy);\n"
            "    float alphaFromMouse = (1.0 - smoothstep(0.0, u_cursorRadius, distFromMouse)) * u_intensity * 0.8;\n"
            "    float finalAlpha = max(alphaFromOrigin, alphaFromMouse);\n"
            "    // Add hole effect - fade in from center\n"
            "    float holeVisibility = smoothstep(u_holeStart, u_holeEnd, distFromOrigin);\n"
            "    FragColor = vec4(u_gridColor, finalAlpha * holeVisibility);\n"
            "}\n";
        
        axisVertexShaderSource =
            "#version 330 core\n"
            "layout (location = 0) in vec3 aPos;\n"
            "layout (location = 1) in vec4 aBaseColorAlpha;\n"
            "uniform mat4 model;\n"
            "uniform mat4 view;\n"
            "uniform mat4 projection;\n"
            "out vec3 vWorldPos;\n"
            "out vec4 vBaseColor;\n"
            "void main() {\n"
            "    vWorldPos = (model * vec4(aPos, 1.0)).xyz;\n"
            "    vBaseColor = aBaseColorAlpha;\n"
            "    gl_Position = projection * view * vec4(vWorldPos, 1.0);\n"
            "}\n";

        axisFragmentShaderSource =
            "#version 330 core\n"
            "out vec4 FragColor;\n"
            "in vec3 vWorldPos;\n"
            "in vec4 vBaseColor;\n"
            "uniform vec3 u_cursorWorldPos;\n"
            "uniform float u_cursorRadius;\n"
            "uniform float u_intensity;\n"
            "uniform float u_fadeStart;\n"
            "uniform float u_fadeEnd;\n"
            "uniform float u_holeStart;\n"
            "uniform float u_holeEnd;\n"
            "uniform vec4 u_positiveFadeColor;\n"
            "uniform vec4 u_negativeFadeColor;\n"
            "void main() {\n"
            "    float distFromOrigin = length(vWorldPos);\n"
            "    float distFromMouse = distance(vWorldPos, u_cursorWorldPos);\n"
            "    float mouseRevealFactor = (1.0 - smoothstep(0.0, u_cursorRadius, distFromMouse)) * u_intensity;\n"
            "    vec3 finalColor;\n"
            "    float finalAlpha;\n"
            "    // Check if it's a negative axis (alpha < 0)\n"
            "    if (vBaseColor.a < 0.0) { \n"
            "        vec3 fadedColor = mix(vBaseColor.rgb, u_negativeFadeColor.rgb, 1.0);\n"
            "        finalColor = mix(fadedColor, vBaseColor.rgb, mouseRevealFactor);\n"
            "        finalAlpha = mix(abs(vBaseColor.a), 1.0, mouseRevealFactor);\n"
            "    } else {\n"
            "        float fadeToWhiteFactor = smoothstep(u_fadeStart, u_fadeEnd, distFromOrigin);\n"
            "        vec3 fadedColor = mix(vBaseColor.rgb, u_positiveFadeColor.rgb, fadeToWhiteFactor);\n"
            "        finalColor = mix(fadedColor, vBaseColor.rgb, mouseRevealFactor);\n"
            "        finalAlpha = mix(vBaseColor.a, 1.0, mouseRevealFactor);\n"
            "    }\n"
            "    // Add hole effect - fade in from center\n"
            "    float holeVisibility = smoothstep(u_holeStart, u_holeEnd, distFromOrigin);\n"
            "    FragColor = vec4(finalColor, finalAlpha * holeVisibility);\n"
            "}\n";

        unlitVertexShaderSource =
            "#version 330 core\n"
            "layout (location = 0) in vec3 aPos;\n"
            "uniform mat4 model;\n"
            "uniform mat4 view;\n"
            "uniform mat4 projection;\n"
            "out vec3 vWorldPos;\n"
            "void main() {\n"
            "    vWorldPos = vec3(model * vec4(aPos, 1.0));\n"
            "    gl_Position = projection * view * vec4(vWorldPos, 1.0);\n"
            "}\n";

        unlitFragmentShaderSource =
            "#version 330 core\n"
            "out vec4 FragColor;\n"
            "uniform vec3 u_baseColor;\n"
            "uniform vec3 u_fadeColor;\n"
            "uniform vec3 u_cursorWorldPos;\n"
            "uniform float u_cursorRadius;\n"
            "uniform float u_intensity;\n"
            "uniform float u_fadeStart;\n"
            "uniform float u_fadeEnd;\n"
            "in vec3 vWorldPos;\n"
            "void main() {\n"
            "    float distFromOrigin = length(vWorldPos);\n"
            "    float fadeToWhiteFactor = smoothstep(u_fadeStart, u_fadeEnd, distFromOrigin);\n"
            "    vec3 fadedColor = mix(u_baseColor, u_fadeColor, fadeToWhiteFactor);\n"
            "    float distFromMouse = distance(vWorldPos, u_cursorWorldPos);\n"
            "    float mouseRevealFactor = (1.0 - smoothstep(0.0, u_cursorRadius, distFromMouse)) * u_intensity;\n"
            "    vec3 finalColor = mix(fadedColor, u_baseColor, mouseRevealFactor);\n"
            "    FragColor = vec4(finalColor, 1.0);\n"
            "}\n";

        splatVertexShaderSource =
            "#version 330 core\n"
            "layout (location = 0) in vec3 aPos;\n"
            "layout (location = 1) in vec2 aTexCoord;\n"
            "uniform mat4 model;\n"
            "uniform mat4 view;\n"
            "uniform mat4 projection;\n"
            "out vec2 TexCoord;\n"
            "void main() {\n"
            "    gl_Position = projection * view * model * vec4(aPos, 1.0);\n"
            "    TexCoord = aTexCoord;\n"
            "}\n";

        splatFragmentShaderSource =
            "#version 330 core\n"
            "out vec4 FragColor;\n"
            "in vec2 TexCoord;\n"
            "uniform vec4 splatColor;\n"
            "uniform float blurStrength;\n"
            "void main() {\n"
            "    float dist = distance(TexCoord, vec2(0.5));\n"
            "    float falloff = exp(-dist * dist * blurStrength);\n"
            "    float finalAlpha = splatColor.a * falloff;\n"
            "    FragColor = vec4(splatColor.rgb, finalAlpha);\n"
            "}\n";

        markerVertexShaderSource =
            "#version 330 core\n"
            "layout (location = 0) in vec2 aPosModel;\n"
            "uniform vec3 u_WorldPos;\n"
            "uniform float u_ScreenSize;\n"
            "uniform mat4 u_ViewMatrix;\n"
            "uniform mat4 u_ProjMatrix;\n"
            "uniform vec2 u_ViewportSize;\n"
            "void main() {\n"
            "    vec4 worldCenter = vec4(u_WorldPos, 1.0);\n"
            "    vec4 clipCenter = u_ProjMatrix * u_ViewMatrix * worldCenter;\n"
            "    if (clipCenter.w <= 0.0) { gl_Position = vec4(2.0, 2.0, 2.0, 1.0); return; }\n" // Cull if behind camera
            "    vec2 scaleFactor = (vec2(u_ScreenSize) / u_ViewportSize) * 2.0 * clipCenter.w;\n"
            "    vec4 clipOffset = vec4(aPosModel * scaleFactor, 0.0, 0.0);\n"
            "    gl_Position = clipCenter + clipOffset;\n"
            "    if (gl_Position.w <= 0.0) { gl_Position.w = 0.001; }\n" // Ensure w is positive for perspective divide
            "}\n";

        markerFragmentShaderSource =
            "#version 330 core\n"
            "out vec4 FragColor;\n"
            "uniform vec4 u_Color;\n"
            "void main() {\n"
            "    FragColor = u_Color;\n"
            "}\n";

        dashedLineVertexShaderSource =
            "#version 330 core\n"
            "layout (location = 0) in vec3 aPos;\n"
            "layout (location = 1) in float aDist;\n" // Distance from start of line
            "uniform mat4 model;\n"
            "uniform mat4 view;\n"
            "uniform mat4 projection;\n"
            "out float vDist;\n"
            "void main() {\n"
            "    gl_Position = projection * view * model * vec4(aPos, 1.0);\n"
            "    vDist = aDist;\n"
            "}\n";
        
        dashedLineFragmentShaderSource =
            "#version 330 core\n"
            "out vec4 FragColor;\n"
            "in float vDist;\n"
            "uniform vec4 u_Color;\n"
            "uniform float u_DashSize;\n"
            "uniform float u_GapSize;\n"
            "void main() {\n"
            "    float patternLength = u_DashSize + u_GapSize;\n"
            "    if (patternLength < 0.001) {\n"
            "        FragColor = u_Color;\n"
            "        return;\n"
            "    }\n"
            "    float modularDist = mod(vDist, patternLength);\n"
            "    if (modularDist > u_DashSize) {\n"
            "        discard;\n"
            "    }\n"
            "    FragColor = u_Color;\n"
            "}\n";

        selectionBoxVertexShaderSource =
            "#version 330 core\n"
            "layout (location = 0) in vec2 aPos;\n" // Input is already in NDC
            "void main() {\n"
            "    gl_Position = vec4(aPos.x, aPos.y, 0.0, 1.0);\n"
            "}\n";

        selectionBoxFragmentShaderSource =
            "#version 330 core\n"
            "out vec4 FragColor;\n"
            "uniform vec4 u_Color;\n"
            "void main() {\n"
            "    FragColor = u_Color;\n"
            "}\n";
            
        vrMenuWidgetVertexShaderSource =
            "#version 330 core\n"
            "layout (location = 0) in vec3 aPos; // Quad from -0.5 to 0.5 on XY plane\n"
            "uniform mat4 projection;\n"
            "uniform mat4 view;\n"
            "uniform mat4 model;\n"
            "out vec2 vUv;\n"
            "void main() {\n"
            "    vUv = aPos.xy;\n"
            "    gl_Position = projection * view * model * vec4(aPos, 1.0);\n"
            "}\n";
        vrMenuWidgetFragmentShaderSource =
            "#version 330 core\n"
            "out vec4 FragColor;\n"
            "in vec2 vUv;\n"
            "\n"
            "uniform vec3 u_baseColor;\n"
            "uniform float u_aberrationAmount;\n"
            "uniform float u_globalAlpha;\n"
            "uniform vec3 u_aberrationColor1;\n"
            "uniform vec3 u_aberrationColor2;\n"
            "\n"
            "const float edgeSoftness = 0.4;\n"
            "\n"
            "void main() {\n"
            "    // Red layer (custom color 1)\n"
            "    vec2 offsetR = vec2(1.0, 1.0) * u_aberrationAmount;\n"
            "    float distR = length(vUv - offsetR);\n"
            "    float alphaR = (1.0 - smoothstep(0.5 - edgeSoftness, 0.5, distR)) * 0.6;\n"
            "    vec4 colorR = vec4(u_aberrationColor1, alphaR);\n"
            "\n"
            "    // Blue layer (custom color 2)\n"
            "    vec2 offsetB = vec2(-1.0, -1.0) * u_aberrationAmount;\n"
            "    float distB = length(vUv - offsetB);\n"
            "    float alphaB = (1.0 - smoothstep(0.5 - edgeSoftness, 0.5, distB)) * 0.7;\n"
            "    vec4 colorB = vec4(u_aberrationColor2, alphaB);\n"
            "\n"
            "    // Base layer\n"
            "    float distBase = length(vUv);\n"
            "    float alphaBase = 1.0 - smoothstep(0.5 - edgeSoftness, 0.5, distBase);\n"
            "    vec4 colorBase = vec4(u_baseColor, alphaBase);\n"
            "\n"
            "    // Additive blending for a glow effect\n"
            "    vec3 finalRGB = colorBase.rgb * colorBase.a + colorR.rgb * colorR.a + colorB.rgb * colorB.a;\n"
            "    float finalAlpha = max(colorBase.a, max(colorR.a, colorB.a));\n"
            "\n"
            "    FragColor = vec4(finalRGB, finalAlpha * u_globalAlpha);\n"
            "}\n";
    }
    Renderer::~Renderer() { Cleanup(); }
    bool Renderer::Initialize() { std::cout << "Renderer: Initializing..." << std::endl; GLfloat range[2] = { 1.0f, 1.0f }; glGetFloatv(GL_ALIASED_LINE_WIDTH_RANGE, range); maxLineWidth = std::max(1.0f, range[1]); std::cout << "Renderer: Supported ALIASED Line Width Range: [" << range[0] << ", " << maxLineWidth << "]" << std::endl; if (!CreateShaderPrograms()) return false; if (!CreateGridResources()) return false; if (!CreateAxesResources()) return false; if (!CreateUserLinesResources()) return false; if (!CreateMarkerResources()) return false; if (!CreatePreviewResources()) return false; if (!CreatePreviewLineResources()) return false; if (!CreatePreviewOutlineResources()) return false; if (!CreateSelectionBoxResources()) return false; if (!CreateVRPointerResources()) return false; if (!CreateSplatResources()) return false; if (!CreateGhostMeshResources()) return false; glEnable(GL_DEPTH_TEST); glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); std::cout << "Renderer: Initialization successful." << std::endl; return true; }
    void Renderer::SetViewport(int x, int y, int width, int height) { if (width > 0 && height > 0) { glViewport(x, y, width, height); } }
    
    void Renderer::RenderFrame(
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
        uint64_t previewObjectId,
        const glm::mat4& previewTransform,
        // --- NEW: Overrides for dynamic objects like controllers ---
        const std::map<uint64_t, glm::mat4>& transformOverrides,
        const std::map<uint64_t, glm::vec3>& colorOverrides,
        const std::map<uint64_t, bool>& unlitOverrides
    ) {
        if (viewportWidth <= 0 || viewportHeight <= 0) return; glm::mat4 identityModel = glm::mat4(1.0f);
        
        float distanceToCamera = glm::length(viewPos);
        const float referenceDistance = 30.0f;
        float distanceScale = distanceToCamera / referenceDistance;
        
        // --- 1. OPAQUE PASS ---
        if (objectShaderProgram != 0 && scene) {
            glUseProgram(objectShaderProgram);
            glUniformMatrix4fv(glGetUniformLocation(objectShaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
            glUniformMatrix4fv(glGetUniformLocation(objectShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
            glm::vec3 lightDir = -glm::normalize(glm::vec3(glm::inverse(view)[2]));
            glUniform3fv(glGetUniformLocation(objectShaderProgram, "lightDir"), 1, glm::value_ptr(lightDir));
            glUniform3fv(glGetUniformLocation(objectShaderProgram, "lightColor"), 1, glm::value_ptr(lightColor));
            glUniform1f(glGetUniformLocation(objectShaderProgram, "ambientStrength"), ambientStrength);
            glUniform3fv(glGetUniformLocation(objectShaderProgram, "viewPos"), 1, glm::value_ptr(viewPos));
            for (const auto* obj : scene->get_all_objects()) {
                if (obj && obj->vao != 0 && obj->index_count > 0) {
                    
                    // Skip the object if it's being moved by the MoveTool (handled by ghost mesh)
                    if (obj->get_id() == previewObjectId) { 
                        continue;
                    }

                    // 1. Set Transform (Model Matrix)
                    glm::mat4 modelMatrix = identityModel;
                    auto transformIt = transformOverrides.find(obj->get_id());
                    if (transformIt != transformOverrides.end()) {
                        modelMatrix = transformIt->second;
                    }
                    glUniformMatrix4fv(glGetUniformLocation(objectShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(modelMatrix));
                    
                    // 2. Set Color
                    glm::vec3 currentColor = defaultObjectColor;
                    auto colorIt = colorOverrides.find(obj->get_id());
                    if (colorIt != colorOverrides.end()) {
                        currentColor = colorIt->second;
                    }
                    glUniform3fv(glGetUniformLocation(objectShaderProgram, "objectColor"), 1, glm::value_ptr(currentColor));
                    
                    // 3. Set Lighting
                    bool isUnlit = false;
                    auto unlitIt = unlitOverrides.find(obj->get_id());
                    if (unlitIt != unlitOverrides.end()) {
                        isUnlit = unlitIt->second;
                    }
                    glUniform1i(glGetUniformLocation(objectShaderProgram, "u_unlit"), isUnlit);
                    glUniform1f(glGetUniformLocation(objectShaderProgram, "overrideAlpha"), 1.0f);
                    
                    // Skip rendering special markers in this main pass
                    const auto& name = obj->get_name();
                    if (name != "CenterMarker" && name != "UnitCapsuleMarker10m" && name != "UnitCapsuleMarker5m") {
                         glBindVertexArray(obj->vao);
                         glDrawElements(GL_TRIANGLES, obj->index_count, GL_UNSIGNED_INT, 0);
                         glBindVertexArray(0);
                    }
                }
            }
        }

        // --- Render Ghost Mesh (as opaque object with boundary wireframe) ---
        // ЗАМЕНИТЕ ВЕСЬ СТАРЫЙ БЛОК "Render Ghost Mesh" НА ЭТОТ
        if (ghostMeshVAO != 0 && ghostMeshTriangleIndexCount > 0) {
            glUseProgram(objectShaderProgram);
            glUniformMatrix4fv(glGetUniformLocation(objectShaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
            glUniformMatrix4fv(glGetUniformLocation(objectShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
            glUniformMatrix4fv(glGetUniformLocation(objectShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(identityModel));
            
            // 1. Solid fill pass
            glEnable(GL_POLYGON_OFFSET_FILL);
            glPolygonOffset(1.0f, 1.0f); // Push the solid fill back slightly
            glUniform1i(glGetUniformLocation(objectShaderProgram, "u_unlit"), 0); // Lighting ON
            glUniform1f(glGetUniformLocation(objectShaderProgram, "overrideAlpha"), 1.0f);
            glUniform3fv(glGetUniformLocation(objectShaderProgram, "objectColor"), 1, glm::value_ptr(defaultObjectColor));
            
            glBindVertexArray(ghostMeshVAO);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ghostMeshEBO_triangles);
            glDrawElements(GL_TRIANGLES, ghostMeshTriangleIndexCount, GL_UNSIGNED_INT, 0);
            
            glDisable(GL_POLYGON_OFFSET_FILL);
        
            // 2. Wireframe overlay pass (if there are lines to draw)
            if (ghostMeshLineIndexCount > 0) {
                glUniform1i(glGetUniformLocation(objectShaderProgram, "u_unlit"), 1); // Lighting OFF for pure white
                glUniform3fv(glGetUniformLocation(objectShaderProgram, "objectColor"), 1, glm::value_ptr(glm::vec3(1.0f))); // White wireframe
                glLineWidth(1.5f);
                
                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ghostMeshEBO_lines);
                glDrawElements(GL_LINES, ghostMeshLineIndexCount, GL_UNSIGNED_INT, 0);
                
                glLineWidth(1.0f);
                glUniform1i(glGetUniformLocation(objectShaderProgram, "u_unlit"), 0); // Reset lighting state
            }
            
            glBindVertexArray(0);
        }

        // --- 2. TRANSPARENT / OVERLAY PASS ---
        glDepthMask(GL_FALSE);
        
        if (objectShaderProgram != 0 && scene) {
            glUseProgram(objectShaderProgram);
            glUniformMatrix4fv(glGetUniformLocation(objectShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(identityModel));
            glUniform1f(glGetUniformLocation(objectShaderProgram, "overrideAlpha"), 1.0f);

            if (hoveredObjId != 0 && !hoveredFaceTriangleIndices.empty()) {
                bool isSameAsSelected = (hoveredObjId == selectedObjId) && (hoveredFaceTriangleIndices == selectedTriangleIndices);
                if (!isSameAsSelected) {
                    Urbaxio::Engine::SceneObject* hoveredObj = scene->get_object_by_id(hoveredObjId);
                     if (hoveredObj && hoveredObj->vao != 0) {
                        glUniform3fv(glGetUniformLocation(objectShaderProgram, "objectColor"), 1, glm::value_ptr(hoverHighlightColor));
                        glEnable(GL_POLYGON_OFFSET_FILL);
                        glPolygonOffset(-2.0f, -2.0f);
                        glBindVertexArray(hoveredObj->vao);
                        for (size_t baseIndex : hoveredFaceTriangleIndices) {
                             if (baseIndex + 2 < hoveredObj->get_mesh_buffers().indices.size()) {
                                 glDrawElements(GL_TRIANGLES, 3, GL_UNSIGNED_INT, (void*)(baseIndex * sizeof(unsigned int)));
                             }
                        }
                        glBindVertexArray(0);
                        glDisable(GL_POLYGON_OFFSET_FILL);
                     }
                }
            }
            if (selectedObjId != 0 && !selectedTriangleIndices.empty()) {
                Urbaxio::Engine::SceneObject* selectedObj = scene->get_object_by_id(selectedObjId);
                if (selectedObj && selectedObj->vao != 0) {
                    glUniform3fv(glGetUniformLocation(objectShaderProgram, "objectColor"), 1, glm::value_ptr(selectionHighlightColor));
                    glEnable(GL_POLYGON_OFFSET_FILL);
                    glPolygonOffset(-1.0f, -1.0f);
                    glBindVertexArray(selectedObj->vao);
                    for (size_t baseIndex : selectedTriangleIndices) {
                        if (baseIndex + 2 < selectedObj->get_mesh_buffers().indices.size()) {
                             glDrawElements(GL_TRIANGLES, 3, GL_UNSIGNED_INT, (void*)(baseIndex * sizeof(unsigned int)));
                        }
                    }
                    glBindVertexArray(0);
                    glDisable(GL_POLYGON_OFFSET_FILL);
                }
            }
            if (previewVertexCount > 0) {
                glUniform3fv(glGetUniformLocation(objectShaderProgram, "objectColor"), 1, glm::value_ptr(hoverHighlightColor));
                glUniform1f(glGetUniformLocation(objectShaderProgram, "overrideAlpha"), 0.5f);
                glBindVertexArray(previewVAO);
                glDrawArrays(GL_TRIANGLES, 0, previewVertexCount);
                glBindVertexArray(0);
                glUniform1f(glGetUniformLocation(objectShaderProgram, "overrideAlpha"), 1.0f);
            }
        }

        if (showGrid && gridVAO != 0 && gridShaderProgram != 0) {
            glLineWidth(1.0f);
            glUseProgram(gridShaderProgram);
            glUniformMatrix4fv(glGetUniformLocation(gridShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(identityModel));
            glUniformMatrix4fv(glGetUniformLocation(gridShaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
            glUniformMatrix4fv(glGetUniformLocation(gridShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
            glUniform3fv(glGetUniformLocation(gridShaderProgram, "u_gridColor"), 1, glm::value_ptr(gridColor));
            glUniform3fv(glGetUniformLocation(gridShaderProgram, "u_cursorWorldPos"), 1, glm::value_ptr(cursorWorldPos));
            glUniform1f(glGetUniformLocation(gridShaderProgram, "u_cursorRadius"), cursorRadius);
            glUniform1f(glGetUniformLocation(gridShaderProgram, "u_intensity"), intensity);
            const float baseHoleStart = 0.25f, baseHoleEnd = 0.75f;
            glUniform1f(glGetUniformLocation(gridShaderProgram, "u_holeStart"), baseHoleStart * distanceScale);
            glUniform1f(glGetUniformLocation(gridShaderProgram, "u_holeEnd"), baseHoleEnd * distanceScale);
            glBindVertexArray(gridVAO);
            glDrawArrays(GL_LINES, 0, gridVertexCount);
            glBindVertexArray(0);
        }

        // --- MODIFIED: Draw static and moving lines separately ---
        if (simpleLineShaderProgram != 0) {
            glLineWidth(2.0f);
            glUseProgram(simpleLineShaderProgram);
            glUniformMatrix4fv(glGetUniformLocation(simpleLineShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(identityModel));
            glUniformMatrix4fv(glGetUniformLocation(simpleLineShaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
            glUniformMatrix4fv(glGetUniformLocation(simpleLineShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

            if (userLinesVAO != 0 && userLinesVertexCount > 0) {
                glBindVertexArray(userLinesVAO);
                glDrawArrays(GL_LINES, 0, userLinesVertexCount);
            }
            
            glBindVertexArray(0);
            glLineWidth(1.0f);
        }
        if (previewLineEnabled && previewLineVAO != 0 && simpleLineShaderProgram != 0) {
            glLineWidth(1.0f);
            glUseProgram(simpleLineShaderProgram);
            glUniformMatrix4fv(glGetUniformLocation(simpleLineShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(identityModel));
            glUniformMatrix4fv(glGetUniformLocation(simpleLineShaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
            glUniformMatrix4fv(glGetUniformLocation(simpleLineShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
            glBindVertexArray(previewLineVAO);
            glDrawArrays(GL_LINES, 0, 2);
            glBindVertexArray(0);
        }

        if (previewOutlineVertexCount > 0) {
            glLineWidth(1.5f);
            glUseProgram(dashedLineShaderProgram);
            glUniformMatrix4fv(glGetUniformLocation(dashedLineShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(identityModel));
            glUniformMatrix4fv(glGetUniformLocation(dashedLineShaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
            glUniformMatrix4fv(glGetUniformLocation(dashedLineShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
            glUniform4f(glGetUniformLocation(dashedLineShaderProgram, "u_Color"), 1.0f, 1.0f, 1.0f, 0.8f);
            glUniform1f(glGetUniformLocation(dashedLineShaderProgram, "u_DashSize"), 0.2f);
            glUniform1f(glGetUniformLocation(dashedLineShaderProgram, "u_GapSize"), 0.1f);
            glBindVertexArray(previewOutlineVAO);
            glDrawArrays(GL_LINES, 0, previewOutlineVertexCount);
            glBindVertexArray(0);
            glLineWidth(1.0f);
        }

        glLineWidth(1.0f);

        // --- NEW: Draw VR Pointer Ray ---
        if (vrPointerEnabled && vrPointerVAO != 0 && simpleLineShaderProgram != 0) {
            glLineWidth(2.0f);
            glUseProgram(simpleLineShaderProgram);
            glUniformMatrix4fv(glGetUniformLocation(simpleLineShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(identityModel));
            glUniformMatrix4fv(glGetUniformLocation(simpleLineShaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
            glUniformMatrix4fv(glGetUniformLocation(simpleLineShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
            glBindVertexArray(vrPointerVAO);
            glDrawArrays(GL_LINES, 0, 2);
            glBindVertexArray(0);
            glLineWidth(1.0f);
        }
        
        if (unlitShaderProgram != 0 && scene) {
            glUseProgram(unlitShaderProgram);
            glUniformMatrix4fv(glGetUniformLocation(unlitShaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
            glUniformMatrix4fv(glGetUniformLocation(unlitShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
            glUniform3fv(glGetUniformLocation(unlitShaderProgram, "u_cursorWorldPos"), 1, glm::value_ptr(cursorWorldPos));
            glUniform1f(glGetUniformLocation(unlitShaderProgram, "u_cursorRadius"), cursorRadius);
            glUniform1f(glGetUniformLocation(unlitShaderProgram, "u_intensity"), intensity);
            Urbaxio::Engine::SceneObject* center_marker = nullptr, *capsule_marker_10m_template = nullptr, *capsule_marker_5m_template = nullptr;
            for(auto* obj : scene->get_all_objects()){ if(obj) { const auto& name = obj->get_name(); if(name == "CenterMarker") center_marker = obj; else if (name == "UnitCapsuleMarker10m") capsule_marker_10m_template = obj; else if (name == "UnitCapsuleMarker5m") capsule_marker_5m_template = obj; } }
            if (center_marker && center_marker->vao != 0) {
                 float scale = (distanceToCamera / referenceDistance) * 0.7f;
                 glUniform1f(glGetUniformLocation(unlitShaderProgram, "u_fadeStart"), 0.0); glUniform1f(glGetUniformLocation(unlitShaderProgram, "u_fadeEnd"), 0.0);
                 glm::mat4 modelMatrix = glm::scale(glm::mat4(1.0f), glm::vec3(scale));
                 glUniformMatrix4fv(glGetUniformLocation(unlitShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(modelMatrix));
                 glUniform3f(glGetUniformLocation(unlitShaderProgram, "u_baseColor"), 1.0f, 1.0f, 1.0f);
                 glUniform3f(glGetUniformLocation(unlitShaderProgram, "u_fadeColor"), 1.0f, 1.0f, 1.0f);
                 glBindVertexArray(center_marker->vao);
                 glDrawElements(GL_TRIANGLES, center_marker->index_count, GL_UNSIGNED_INT, 0);
            }
            if (showAxes && capsule_marker_10m_template && capsule_marker_10m_template->vao != 0 && capsule_marker_5m_template && capsule_marker_5m_template->vao != 0) {
                const float markerBaseScale = 0.15f, maxMarkerDist = 100, maxMarkerWorldSize = 0.5f;
                const float baseFadeStart = 0.5f, baseFadeEnd = 4.0f;
                glUniform1f(glGetUniformLocation(unlitShaderProgram, "u_fadeStart"), baseFadeStart * distanceScale);
                glUniform1f(glGetUniformLocation(unlitShaderProgram, "u_fadeEnd"), baseFadeEnd * distanceScale);
                struct AxisInfo { glm::vec3 dir; glm::vec4 color; }; std::vector<AxisInfo> axes_info = { {glm::vec3(1,0,0), axisColorX}, {glm::vec3(0,1,0), axisColorY}, {glm::vec3(0,0,1), axisColorZ} };
                glUniform3fv(glGetUniformLocation(unlitShaderProgram, "u_fadeColor"), 1, glm::value_ptr(glm::vec3(positiveAxisFadeColor)));
                for(const auto& axis : axes_info) {
                    glUniform3fv(glGetUniformLocation(unlitShaderProgram, "u_baseColor"), 1, glm::value_ptr(glm::vec3(axis.color)));
                    for(int i = 5; i <= maxMarkerDist; i += 5) {
                        glm::vec3 position = axis.dir * (float)i; float distToMarker = glm::length(viewPos - position); float scale = (distToMarker / referenceDistance) * markerBaseScale;
                        if (scale > maxMarkerWorldSize) scale = maxMarkerWorldSize;
                        glm::mat4 translationMatrix = glm::translate(glm::mat4(1.0f), position);
                        Urbaxio::Engine::SceneObject* current_marker_template = (i % 10 == 0) ? capsule_marker_10m_template : capsule_marker_5m_template;
                        glm::quat rotation; if (glm::abs(axis.dir.z) > 0.99) { rotation = glm::angleAxis(glm::radians(90.0f), glm::vec3(1,0,0)); rotation = glm::angleAxis(glm::radians(45.0f), glm::vec3(0,0,1)) * rotation; } else { rotation = glm::angleAxis(glm::radians(90.0f), glm::vec3(1,0,0)); rotation = glm::angleAxis(glm::atan(axis.dir.y, axis.dir.x), glm::vec3(0,0,1)) * rotation; }
                        glm::mat4 modelMatrix = translationMatrix * glm::mat4_cast(rotation) * glm::scale(glm::mat4(1.0f), glm::vec3(scale));
                        glUniformMatrix4fv(glGetUniformLocation(unlitShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(modelMatrix));
                        glBindVertexArray(current_marker_template->vao);
                        glDrawElements(GL_TRIANGLES, current_marker_template->index_count, GL_UNSIGNED_INT, 0);
                    }
                }
            }
            glBindVertexArray(0);
        }
        if (showAxes && axesVAO != 0 && axisShaderProgram != 0) {
            UpdateAxesVBO(axisColorX, axisColorY, axisColorZ, positiveAxisFadeColor, negativeAxisFadeColor);
            glUseProgram(axisShaderProgram);
            glUniformMatrix4fv(glGetUniformLocation(axisShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(identityModel));
            glUniformMatrix4fv(glGetUniformLocation(axisShaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
            glUniformMatrix4fv(glGetUniformLocation(axisShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
            glUniform3fv(glGetUniformLocation(axisShaderProgram, "u_cursorWorldPos"), 1, glm::value_ptr(cursorWorldPos));
            glUniform1f(glGetUniformLocation(axisShaderProgram, "u_cursorRadius"), cursorRadius);
            glUniform1f(glGetUniformLocation(axisShaderProgram, "u_intensity"), intensity);
            const float baseFadeStart = 0.5f, baseFadeEnd = 4.0f, baseHoleStart = 0.25f, baseHoleEnd = 0.75f;
            glUniform1f(glGetUniformLocation(axisShaderProgram, "u_fadeStart"), baseFadeStart * distanceScale);
            glUniform1f(glGetUniformLocation(axisShaderProgram, "u_fadeEnd"), baseFadeEnd * distanceScale);
            glUniform1f(glGetUniformLocation(axisShaderProgram, "u_holeStart"), baseHoleStart * distanceScale);
            glUniform1f(glGetUniformLocation(axisShaderProgram, "u_holeEnd"), baseHoleEnd * distanceScale);
            glUniform4fv(glGetUniformLocation(axisShaderProgram, "u_positiveFadeColor"), 1, glm::value_ptr(positiveAxisFadeColor));
            glUniform4fv(glGetUniformLocation(axisShaderProgram, "u_negativeFadeColor"), 1, glm::value_ptr(negativeAxisFadeColor));
            glBindVertexArray(axesVAO);
            glLineWidth(negAxisLineWidth); glDrawArrays(GL_LINES, 2, 2); glDrawArrays(GL_LINES, 6, 2); glDrawArrays(GL_LINES, 10, 2);
            glLineWidth(axisLineWidth); glDrawArrays(GL_LINES, 0, 2); glDrawArrays(GL_LINES, 4, 2); glDrawArrays(GL_LINES, 8, 2);
            glBindVertexArray(0);
        }

        // --- FIX: Disable depth test for snap markers ---
        glDisable(GL_DEPTH_TEST);
        DrawSnapMarker(currentSnap, view, projection, viewportWidth, viewportHeight);
        glEnable(GL_DEPTH_TEST);

        glDepthMask(GL_TRUE);

        glLineWidth(1.0f);
        glUseProgram(0);
        if (imguiDrawData) ImGui_ImplOpenGL3_RenderDrawData(imguiDrawData);
    }
    bool Renderer::CreateShaderPrograms() {
        // Object Shader
        { GLuint vs = CompileShader(GL_VERTEX_SHADER, objectVertexShaderSource); GLuint fs = CompileShader(GL_FRAGMENT_SHADER, objectFragmentShaderSource); if (vs != 0 && fs != 0) objectShaderProgram = LinkShaderProgram(vs, fs); if (objectShaderProgram == 0) return false; std::cout << "Renderer: Object shader program created." << std::endl; }
        // Simple Line Shader
        { GLuint vs = CompileShader(GL_VERTEX_SHADER, simpleLineVertexShaderSource); GLuint fs = CompileShader(GL_FRAGMENT_SHADER, simpleLineFragmentShaderSource); if (vs != 0 && fs != 0) simpleLineShaderProgram = LinkShaderProgram(vs, fs); if (simpleLineShaderProgram == 0) return false; std::cout << "Renderer: Simple Line shader program created." << std::endl; }
        // Grid Shader
        { GLuint vs = CompileShader(GL_VERTEX_SHADER, gridVertexShaderSource); GLuint fs = CompileShader(GL_FRAGMENT_SHADER, gridFragmentShaderSource); if (vs != 0 && fs != 0) gridShaderProgram = LinkShaderProgram(vs, fs); if (gridShaderProgram == 0) return false; std::cout << "Renderer: Grid shader program created." << std::endl; }
        // Axis Shader
        { GLuint vs = CompileShader(GL_VERTEX_SHADER, axisVertexShaderSource); GLuint fs = CompileShader(GL_FRAGMENT_SHADER, axisFragmentShaderSource); if (vs != 0 && fs != 0) axisShaderProgram = LinkShaderProgram(vs, fs); if (axisShaderProgram == 0) return false; std::cout << "Renderer: Axis shader program created." << std::endl; }
        // Unlit Shader
        { GLuint vs = CompileShader(GL_VERTEX_SHADER, unlitVertexShaderSource); GLuint fs = CompileShader(GL_FRAGMENT_SHADER, unlitFragmentShaderSource); if (vs != 0 && fs != 0) unlitShaderProgram = LinkShaderProgram(vs, fs); if (unlitShaderProgram == 0) return false; std::cout << "Renderer: Unlit shader program created." << std::endl; }
        // Splat Shader
        { GLuint vs = CompileShader(GL_VERTEX_SHADER, splatVertexShaderSource); GLuint fs = CompileShader(GL_FRAGMENT_SHADER, splatFragmentShaderSource); if (vs != 0 && fs != 0) splatShaderProgram = LinkShaderProgram(vs, fs); if (splatShaderProgram == 0) return false; std::cout << "Renderer: Splat shader program created." << std::endl; }
        // Marker Shader
        { GLuint vs = CompileShader(GL_VERTEX_SHADER, markerVertexShaderSource); GLuint fs = CompileShader(GL_FRAGMENT_SHADER, markerFragmentShaderSource); if (vs != 0 && fs != 0) markerShaderProgram = LinkShaderProgram(vs, fs); if (markerShaderProgram == 0) { std::cerr << "Renderer Error: Failed to link marker shader program!" << std::endl; return false; } std::cout << "Renderer: Marker shader program created." << std::endl; }
        // Dashed Line Shader
        { GLuint vs = CompileShader(GL_VERTEX_SHADER, dashedLineVertexShaderSource); GLuint fs = CompileShader(GL_FRAGMENT_SHADER, dashedLineFragmentShaderSource); if (vs != 0 && fs != 0) dashedLineShaderProgram = LinkShaderProgram(vs, fs); if (dashedLineShaderProgram == 0) return false; std::cout << "Renderer: Dashed Line shader program created." << std::endl; }
        // Selection Box Shader
        { GLuint vs = CompileShader(GL_VERTEX_SHADER, selectionBoxVertexShaderSource); GLuint fs = CompileShader(GL_FRAGMENT_SHADER, selectionBoxFragmentShaderSource); if (vs != 0 && fs != 0) selectionBoxShaderProgram = LinkShaderProgram(vs, fs); if (selectionBoxShaderProgram == 0) return false; std::cout << "Renderer: Selection Box shader program created." << std::endl; }
        { GLuint vs = CompileShader(GL_VERTEX_SHADER, vrMenuWidgetVertexShaderSource); GLuint fs = CompileShader(GL_FRAGMENT_SHADER, vrMenuWidgetFragmentShaderSource); if (vs != 0 && fs != 0) vrMenuWidgetShaderProgram = LinkShaderProgram(vs, fs); if (vrMenuWidgetShaderProgram == 0) return false; std::cout << "Renderer: VR Menu Widget shader program created." << std::endl; }
        return true;
    }
    bool Renderer::CreateGridResources() {
        std::vector<float> gridVertices = GenerateGridVertices(gridSizeF, gridSteps);
        if (gridVertices.empty()) {
            std::cerr << "Renderer Error: Failed to generate grid vertices!" << std::endl;
            return false;
        }
        gridVertexCount = static_cast<int>(gridVertices.size() / 3);

        glGenVertexArrays(1, &gridVAO);
        glGenBuffers(1, &gridVBO);
        glBindVertexArray(gridVAO);
        glBindBuffer(GL_ARRAY_BUFFER, gridVBO);
        glBufferData(GL_ARRAY_BUFFER, gridVertices.size() * sizeof(float), gridVertices.data(), GL_STATIC_DRAW);
        // Only position attribute is needed now
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
        std::cout << "Renderer: Grid VAO/VBO created (" << gridVertexCount << " vertices)." << std::endl;
        return gridVAO != 0 && gridVBO != 0;
    }
    bool Renderer::CreateAxesResources() {
        glm::vec4 colorX(0.7f, 0.2f, 0.2f, 1.0f);
        glm::vec4 colorY(0.2f, 0.7f, 0.2f, 1.0f);
        glm::vec4 colorZ(0.2f, 0.2f, 0.7f, 1.0f);
        glm::vec4 posFadeColor(1.0f, 1.0f, 1.0f, 1.0f);
        glm::vec4 negFadeColor(1.0f, 1.0f, 1.0f, 0.4f);

        std::vector<float> verticesX = GenerateSingleAxisVertices(axisLength, glm::vec3(1, 0, 0), colorX, posFadeColor, negFadeColor);
        std::vector<float> verticesY = GenerateSingleAxisVertices(axisLength, glm::vec3(0, 1, 0), colorY, posFadeColor, negFadeColor);
        std::vector<float> verticesZ = GenerateSingleAxisVertices(axisLength, glm::vec3(0, 0, 1), colorZ, posFadeColor, negFadeColor);

        axisVertexCountX = 4;
        axisVertexCountY = 4;
        axisVertexCountZ = 4;
        
        std::vector<float> allAxesVertices;
        allAxesVertices.insert(allAxesVertices.end(), verticesX.begin(), verticesX.end());
        allAxesVertices.insert(allAxesVertices.end(), verticesY.begin(), verticesY.end());
        allAxesVertices.insert(allAxesVertices.end(), verticesZ.begin(), verticesZ.end());

        if (allAxesVertices.empty()) {
            std::cerr << "Renderer Error: Failed to generate axes vertices!" << std::endl;
            return false;
        }

        glGenVertexArrays(1, &axesVAO);
        glGenBuffers(1, &axesVBO);
        glBindVertexArray(axesVAO);
        glBindBuffer(GL_ARRAY_BUFFER, axesVBO);
        glBufferData(GL_ARRAY_BUFFER, allAxesVertices.size() * sizeof(float), allAxesVertices.data(), GL_DYNAMIC_DRAW);
        
        // Position
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        // Color
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
        std::cout << "Renderer: Axes VAO/VBO created." << std::endl;
        return axesVAO != 0 && axesVBO != 0;
    }
    bool Renderer::CreateSplatResources() { /* ... same ... */ std::vector<float> quadVertices = GenerateQuadVertices(2.0f); unsigned int quadIndices[] = { 0, 1, 2, 0, 2, 3 }; glGenVertexArrays(1, &splatVAO); glGenBuffers(1, &splatVBO); glGenBuffers(1, &splatEBO); glBindVertexArray(splatVAO); glBindBuffer(GL_ARRAY_BUFFER, splatVBO); glBufferData(GL_ARRAY_BUFFER, quadVertices.size() * sizeof(float), quadVertices.data(), GL_STATIC_DRAW); glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, splatEBO); glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(quadIndices), quadIndices, GL_STATIC_DRAW); glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0); glEnableVertexAttribArray(0); glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float))); glEnableVertexAttribArray(1); glBindVertexArray(0); std::cout << "Renderer: Splat VAO/VBO/EBO created." << std::endl; return splatVAO != 0 && splatVBO != 0 && splatEBO != 0; }
    bool Renderer::CreateUserLinesResources() { 
        // Static lines
        glGenVertexArrays(1, &userLinesVAO); 
        glGenBuffers(1, &userLinesVBO); 
        glBindVertexArray(userLinesVAO); 
        glBindBuffer(GL_ARRAY_BUFFER, userLinesVBO); 
        glBufferData(GL_ARRAY_BUFFER, 10000 * 7 * sizeof(float), nullptr, GL_DYNAMIC_DRAW); 
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)0); 
        glEnableVertexAttribArray(0); 
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)(3 * sizeof(float))); 
        glEnableVertexAttribArray(1);

        // Moving lines
        glGenVertexArrays(1, &movingLinesVAO);
        glGenBuffers(1, &movingLinesVBO);
        glBindVertexArray(movingLinesVAO);
        glBindBuffer(GL_ARRAY_BUFFER, movingLinesVBO);
        glBufferData(GL_ARRAY_BUFFER, 10000 * 7 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);

        glBindBuffer(GL_ARRAY_BUFFER, 0); 
        glBindVertexArray(0); 
        std::cout << "Renderer: User Lines (static and moving) VAO/VBOs created." << std::endl; 
        return userLinesVAO != 0 && userLinesVBO != 0 && movingLinesVAO != 0 && movingLinesVBO != 0; 
    }
    
    void Renderer::UpdateUserLinesBuffer(const std::map<uint64_t, Engine::Line>& lines, const std::set<uint64_t>& selectedLineIDs, uint64_t movingObjectId, Engine::Scene* scene) {
        if (userLinesVBO == 0) return;
        
        // --- NEW: Completely ignore lines from moving object ---
        std::set<uint64_t> movingLineIds;
        if (movingObjectId != 0 && scene) {
            Engine::SceneObject* obj = scene->get_object_by_id(movingObjectId);
            if (obj) {
                movingLineIds = obj->boundaryLineIDs;
            }
        }

        std::vector<float> staticLineData;
        
        for (const auto& [lineID, line] : lines) {
            // Skip lines that belong to the moving object
            if (movingLineIds.count(lineID)) {
                continue;
            }
            
            glm::vec4 currentColor = userLineColor;
            if (selectedLineIDs.count(lineID)) {
                currentColor = selectedUserLineColor;
            }
            
            staticLineData.insert(staticLineData.end(), {line.start.x, line.start.y, line.start.z, currentColor.r, currentColor.g, currentColor.b, currentColor.a});
            staticLineData.insert(staticLineData.end(), {line.end.x, line.end.y, line.end.z, currentColor.r, currentColor.g, currentColor.b, currentColor.a});
        }
        
        glBindBuffer(GL_ARRAY_BUFFER, userLinesVBO);
        glBufferData(GL_ARRAY_BUFFER, staticLineData.size() * sizeof(float), staticLineData.data(), GL_DYNAMIC_DRAW);
        userLinesVertexCount = static_cast<int>(staticLineData.size() / 7);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
    bool Renderer::CreateMarkerResources() { /* ... same ... */ std::vector<float> circleVertices = GenerateCircleVertices(24); if (circleVertices.empty()) return false; markerVertexCounts[MarkerShape::CIRCLE] = static_cast<int>(circleVertices.size() / 2); glGenVertexArrays(1, &markerVAOs[MarkerShape::CIRCLE]); glGenBuffers(1, &markerVBOs[MarkerShape::CIRCLE]); glBindVertexArray(markerVAOs[MarkerShape::CIRCLE]); glBindBuffer(GL_ARRAY_BUFFER, markerVBOs[MarkerShape::CIRCLE]); glBufferData(GL_ARRAY_BUFFER, circleVertices.size() * sizeof(float), circleVertices.data(), GL_STATIC_DRAW); glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0); glEnableVertexAttribArray(0); glBindVertexArray(0); std::cout << "Renderer: Circle Marker VAO/VBO created (" << markerVertexCounts[MarkerShape::CIRCLE] << " vertices)." << std::endl; std::vector<float> diamondVertices = GenerateDiamondVertices(); if (diamondVertices.empty()) return false; markerVertexCounts[MarkerShape::DIAMOND] = static_cast<int>(diamondVertices.size() / 2); glGenVertexArrays(1, &markerVAOs[MarkerShape::DIAMOND]); glGenBuffers(1, &markerVBOs[MarkerShape::DIAMOND]); glBindVertexArray(markerVAOs[MarkerShape::DIAMOND]); glBindBuffer(GL_ARRAY_BUFFER, markerVBOs[MarkerShape::DIAMOND]); glBufferData(GL_ARRAY_BUFFER, diamondVertices.size() * sizeof(float), diamondVertices.data(), GL_STATIC_DRAW); glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0); glEnableVertexAttribArray(0); glBindVertexArray(0); std::cout << "Renderer: Diamond Marker VAO/VBO created (" << markerVertexCounts[MarkerShape::DIAMOND] << " vertices)." << std::endl; return true; }
    void Renderer::DrawSnapMarker(const SnapResult& snap, const glm::mat4& view, const glm::mat4& proj, int viewportWidth, int viewportHeight) { /* ... same ... */ if (!snap.snapped || markerShaderProgram == 0) return; MarkerShape shape = MarkerShape::CIRCLE; glm::vec4 color = snapMarkerColorPoint; float currentMarkerSize = markerScreenSize; switch (snap.type) { case SnapType::ENDPOINT: case SnapType::ORIGIN: shape = MarkerShape::CIRCLE; color = snapMarkerColorPoint; currentMarkerSize = markerScreenSize; break; case SnapType::MIDPOINT: shape = MarkerShape::CIRCLE; color = snapMarkerColorMidpoint; currentMarkerSize = markerScreenSizeMidpoint; break; case SnapType::ON_EDGE: shape = MarkerShape::DIAMOND; color = snapMarkerColorOnEdge; currentMarkerSize = markerScreenSizeOnEdge; break; case SnapType::AXIS_X: shape = MarkerShape::DIAMOND; color = snapMarkerColorAxisX; currentMarkerSize = markerScreenSize; break; case SnapType::AXIS_Y: shape = MarkerShape::DIAMOND; color = snapMarkerColorAxisY; currentMarkerSize = markerScreenSize; break; case SnapType::AXIS_Z: shape = MarkerShape::DIAMOND; color = snapMarkerColorAxisZ; currentMarkerSize = markerScreenSize; break; case SnapType::ON_FACE: shape = MarkerShape::CIRCLE; color = snapMarkerColorOnEdge; /* Using OnEdge magenta for now, define snapMarkerColorOnFace later */ currentMarkerSize = markerScreenSize; break; default: return; } if (markerVAOs.find(shape) == markerVAOs.end()) return; GLuint vao = markerVAOs[shape]; int vertexCount = markerVertexCounts[shape]; if (vao == 0 || vertexCount == 0) return; glUseProgram(markerShaderProgram); glUniform3fv(glGetUniformLocation(markerShaderProgram, "u_WorldPos"), 1, glm::value_ptr(snap.worldPoint)); glUniform1f(glGetUniformLocation(markerShaderProgram, "u_ScreenSize"), currentMarkerSize); glUniformMatrix4fv(glGetUniformLocation(markerShaderProgram, "u_ViewMatrix"), 1, GL_FALSE, glm::value_ptr(view)); glUniformMatrix4fv(glGetUniformLocation(markerShaderProgram, "u_ProjMatrix"), 1, GL_FALSE, glm::value_ptr(proj)); glUniform2f(glGetUniformLocation(markerShaderProgram, "u_ViewportSize"), (float)viewportWidth, (float)viewportHeight); glUniform4fv(glGetUniformLocation(markerShaderProgram, "u_Color"), 1, glm::value_ptr(color)); glBindVertexArray(vao); if (shape == MarkerShape::CIRCLE) { glDrawArrays(GL_TRIANGLE_FAN, 0, vertexCount); } else if (shape == MarkerShape::DIAMOND) { glLineWidth(2.0f); glDrawArrays(GL_LINE_LOOP, 0, vertexCount -1); } glBindVertexArray(0); glLineWidth(1.0f); }
    void Renderer::Cleanup() {
        std::cout << "Renderer: Cleaning up resources..." << std::endl;
        if (gridVAO != 0) glDeleteVertexArrays(1, &gridVAO); gridVAO = 0; if (gridVBO != 0) glDeleteBuffers(1, &gridVBO); gridVBO = 0;
        if (axesVAO != 0) glDeleteVertexArrays(1, &axesVAO); axesVAO = 0; if (axesVBO != 0) glDeleteBuffers(1, &axesVBO); axesVBO = 0;
        if (splatVAO != 0) glDeleteVertexArrays(1, &splatVAO); splatVAO = 0; if (splatVBO != 0) glDeleteBuffers(1, &splatVBO); splatVBO = 0; if (splatEBO != 0) glDeleteBuffers(1, &splatEBO); splatEBO = 0;
        if (userLinesVAO != 0) glDeleteVertexArrays(1, &userLinesVAO); userLinesVAO = 0; if (userLinesVBO != 0) glDeleteBuffers(1, &userLinesVBO); userLinesVBO = 0;
        if (previewVAO != 0) glDeleteVertexArrays(1, &previewVAO); previewVAO = 0; if (previewVBO != 0) glDeleteBuffers(1, &previewVBO); previewVBO = 0;
        if (previewLineVAO != 0) glDeleteVertexArrays(1, &previewLineVAO); previewLineVAO = 0; if (previewLineVBO != 0) glDeleteBuffers(1, &previewLineVBO); previewLineVBO = 0;
        if (previewOutlineVAO != 0) glDeleteVertexArrays(1, &previewOutlineVAO); previewOutlineVAO = 0; if (previewOutlineVBO != 0) glDeleteBuffers(1, &previewOutlineVBO); previewOutlineVBO = 0;
        if (ghostMeshVAO != 0) { glDeleteVertexArrays(1, &ghostMeshVAO); ghostMeshVAO = 0; }
        if (vrPointerVAO != 0) glDeleteVertexArrays(1, &vrPointerVAO); vrPointerVAO = 0; if (vrPointerVBO != 0) glDeleteBuffers(1, &vrPointerVBO); vrPointerVBO = 0;
        if (ghostMeshVBO_vertices != 0) { glDeleteBuffers(1, &ghostMeshVBO_vertices); ghostMeshVBO_vertices = 0; }
        if (ghostMeshVBO_normals != 0) { glDeleteBuffers(1, &ghostMeshVBO_normals); ghostMeshVBO_normals = 0; }
        if (ghostMeshEBO_triangles != 0) { glDeleteBuffers(1, &ghostMeshEBO_triangles); ghostMeshEBO_triangles = 0; }
        if (ghostMeshEBO_lines != 0) { glDeleteBuffers(1, &ghostMeshEBO_lines); ghostMeshEBO_lines = 0; }
        for (auto const& [shape, vao] : markerVAOs) { if (vao != 0) glDeleteVertexArrays(1, &vao); } markerVAOs.clear();
        for (auto const& [shape, vbo] : markerVBOs) { if (vbo != 0) glDeleteBuffers(1, &vbo); } markerVBOs.clear();
        markerVertexCounts.clear();
        if (objectShaderProgram != 0) glDeleteProgram(objectShaderProgram); objectShaderProgram = 0;
        if (simpleLineShaderProgram != 0) glDeleteProgram(simpleLineShaderProgram); simpleLineShaderProgram = 0;
        if (gridShaderProgram != 0) glDeleteProgram(gridShaderProgram); gridShaderProgram = 0;
        if (axisShaderProgram != 0) glDeleteProgram(axisShaderProgram); axisShaderProgram = 0;
        if (unlitShaderProgram != 0) glDeleteProgram(unlitShaderProgram); unlitShaderProgram = 0;
        if (splatShaderProgram != 0) glDeleteProgram(splatShaderProgram); splatShaderProgram = 0;
        if (markerShaderProgram != 0) glDeleteProgram(markerShaderProgram); markerShaderProgram = 0;
        if (dashedLineShaderProgram != 0) glDeleteProgram(dashedLineShaderProgram); dashedLineShaderProgram = 0;
        if (selectionBoxShaderProgram != 0) glDeleteProgram(selectionBoxShaderProgram); selectionBoxShaderProgram = 0;
        if (vrMenuWidgetShaderProgram != 0) glDeleteProgram(vrMenuWidgetShaderProgram); vrMenuWidgetShaderProgram = 0;
        if (selectionBoxVAO != 0) glDeleteVertexArrays(1, &selectionBoxVAO); selectionBoxVAO = 0; if (selectionBoxVBO != 0) glDeleteBuffers(1, &selectionBoxVBO); selectionBoxVBO = 0;
        std::cout << "Renderer: Resource cleanup finished." << std::endl;
    }

    bool Renderer::CreatePreviewResources() {
        glGenVertexArrays(1, &previewVAO);
        glGenBuffers(1, &previewVBO);
        glBindVertexArray(previewVAO);
        glBindBuffer(GL_ARRAY_BUFFER, previewVBO);
        // Allocate a large buffer for dynamic drawing
        glBufferData(GL_ARRAY_BUFFER, 10000 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
        // Position attribute
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        // Normal attribute
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glBindVertexArray(0);
        std::cout << "Renderer: Preview VAO/VBO created." << std::endl;
        return previewVAO != 0 && previewVBO != 0;
    }

    bool Renderer::CreatePreviewLineResources() {
        glGenVertexArrays(1, &previewLineVAO);
        glGenBuffers(1, &previewLineVBO);
        glBindVertexArray(previewLineVAO);
        glBindBuffer(GL_ARRAY_BUFFER, previewLineVBO);
        // Buffer for a single line (2 vertices, 7 floats each: position + color)
        glBufferData(GL_ARRAY_BUFFER, 2 * 7 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
        // Position attribute
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        // Color attribute
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glBindVertexArray(0);
        previewLineEnabled = false;
        std::cout << "Renderer: Preview Line VAO/VBO created." << std::endl;
        return previewLineVAO != 0 && previewLineVBO != 0;
    }

    bool Renderer::CreatePreviewOutlineResources() {
        glGenVertexArrays(1, &previewOutlineVAO);
        glGenBuffers(1, &previewOutlineVBO);
        glBindVertexArray(previewOutlineVAO);
        glBindBuffer(GL_ARRAY_BUFFER, previewOutlineVBO);
        // Allocate a large buffer for dynamic drawing. 4 floats per vertex (pos.xyz, dist.w)
        glBufferData(GL_ARRAY_BUFFER, 10000 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
        // Position attribute (vec3)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        // Distance attribute (float)
        glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glBindVertexArray(0);
        std::cout << "Renderer: Preview Outline VAO/VBO created." << std::endl;
        return previewOutlineVAO != 0 && previewOutlineVBO != 0;
    }

    bool Renderer::CreateVRPointerResources() {
        glGenVertexArrays(1, &vrPointerVAO);
        glGenBuffers(1, &vrPointerVBO);
        glBindVertexArray(vrPointerVAO);
        glBindBuffer(GL_ARRAY_BUFFER, vrPointerVBO);
        // Buffer for a single line (2 vertices, 7 floats each: position + color)
        glBufferData(GL_ARRAY_BUFFER, 2 * 7 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
        // Position attribute
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        // Color attribute
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glBindVertexArray(0);
        vrPointerEnabled = false;
        std::cout << "Renderer: VR Pointer VAO/VBO created." << std::endl;
        return vrPointerVAO != 0 && vrPointerVBO != 0;
    }

    void Renderer::UpdatePushPullPreview(const Engine::SceneObject& object, const std::vector<size_t>& faceIndices, const glm::vec3& direction, float distance) {
        // 1. Initial checks
        if (faceIndices.empty() || std::abs(distance) < 1e-4) {
            previewVertexCount = 0;
            previewOutlineVertexCount = 0;
            return;
        }
        const auto& mesh = object.get_mesh_buffers();
        if (!object.has_mesh()) {
            previewVertexCount = 0;
            previewOutlineVertexCount = 0;
            return;
        }

        // 2. Find boundary edges and vertices of the selected face patch
        std::map<std::pair<unsigned int, unsigned int>, int> edgeCounts;
        std::set<unsigned int> boundaryVertexIndices;
        for (size_t baseIndex : faceIndices) {
            if (baseIndex + 2 >= mesh.indices.size()) continue;

            unsigned int v_indices[3] = { mesh.indices[baseIndex], mesh.indices[baseIndex + 1], mesh.indices[baseIndex + 2] };
            for (int j = 0; j < 3; ++j) {
                unsigned int v1_idx = v_indices[j];
                unsigned int v2_idx = v_indices[(j + 1) % 3];
                if (v1_idx > v2_idx) std::swap(v1_idx, v2_idx);
                edgeCounts[{v1_idx, v2_idx}]++;
            }
        }

        std::vector<std::pair<unsigned int, unsigned int>> boundaryEdges;
        for (const auto& [edge, count] : edgeCounts) {
            if (count == 1) {
                boundaryEdges.push_back(edge);
                boundaryVertexIndices.insert(edge.first);
                boundaryVertexIndices.insert(edge.second);
            }
        }

        // 3. Generate preview vertices for both solid fill and dashed outline
        std::vector<float> solidVertices;
        std::vector<float> outlineVertices;
        glm::vec3 offset = direction * distance;
        
        // 3a. Generate top cap (for solid fill)
        for (size_t baseIndex : faceIndices) {
            if (baseIndex + 2 >= mesh.indices.size()) continue;
            unsigned int i0 = mesh.indices[baseIndex], i1 = mesh.indices[baseIndex+1], i2 = mesh.indices[baseIndex+2];
            glm::vec3 v0d = glm::vec3(mesh.vertices[i0*3], mesh.vertices[i0*3+1], mesh.vertices[i0*3+2]) + offset;
            glm::vec3 v1d = glm::vec3(mesh.vertices[i1*3], mesh.vertices[i1*3+1], mesh.vertices[i1*3+2]) + offset;
            glm::vec3 v2d = glm::vec3(mesh.vertices[i2*3], mesh.vertices[i2*3+1], mesh.vertices[i2*3+2]) + offset;
            solidVertices.insert(solidVertices.end(), {v0d.x, v0d.y, v0d.z, direction.x, direction.y, direction.z});
            solidVertices.insert(solidVertices.end(), {v1d.x, v1d.y, v1d.z, direction.x, direction.y, direction.z});
            solidVertices.insert(solidVertices.end(), {v2d.x, v2d.y, v2d.z, direction.x, direction.y, direction.z});
        }

        // 3b. Generate side walls (solid fill) from boundary edges
        for (const auto& edge : boundaryEdges) {
            glm::vec3 p1(mesh.vertices[edge.first*3], mesh.vertices[edge.first*3+1], mesh.vertices[edge.first*3+2]);
            glm::vec3 p2(mesh.vertices[edge.second*3], mesh.vertices[edge.second*3+1], mesh.vertices[edge.second*3+2]);
            glm::vec3 p1d = p1 + offset; glm::vec3 p2d = p2 + offset;
            glm::vec3 sideNormal = glm::normalize(glm::cross(p2 - p1, direction));
            solidVertices.insert(solidVertices.end(), {p1.x, p1.y, p1.z, sideNormal.x, sideNormal.y, sideNormal.z});
            solidVertices.insert(solidVertices.end(), {p2.x, p2.y, p2.z, sideNormal.x, sideNormal.y, sideNormal.z});
            solidVertices.insert(solidVertices.end(), {p1d.x, p1d.y, p1d.z, sideNormal.x, sideNormal.y, sideNormal.z});
            solidVertices.insert(solidVertices.end(), {p1d.x, p1d.y, p1d.z, sideNormal.x, sideNormal.y, sideNormal.z});
            solidVertices.insert(solidVertices.end(), {p2.x, p2.y, p2.z, sideNormal.x, sideNormal.y, sideNormal.z});
            solidVertices.insert(solidVertices.end(), {p2d.x, p2d.y, p2d.z, sideNormal.x, sideNormal.y, sideNormal.z});
        }
        
        // 3c. Generate outline edges (dashed)
        // Top and bottom outlines
        for (const auto& edge : boundaryEdges) {
            glm::vec3 p1(mesh.vertices[edge.first*3], mesh.vertices[edge.first*3+1], mesh.vertices[edge.first*3+2]);
            glm::vec3 p2(mesh.vertices[edge.second*3], mesh.vertices[edge.second*3+1], mesh.vertices[edge.second*3+2]);
            float len = glm::distance(p1, p2);
            // Bottom edge
            outlineVertices.insert(outlineVertices.end(), {p1.x, p1.y, p1.z, 0.0f});
            outlineVertices.insert(outlineVertices.end(), {p2.x, p2.y, p2.z, len});
            // Top edge
            glm::vec3 p1d = p1 + offset; glm::vec3 p2d = p2 + offset;
            outlineVertices.insert(outlineVertices.end(), {p1d.x, p1d.y, p1d.z, 0.0f});
            outlineVertices.insert(outlineVertices.end(), {p2d.x, p2d.y, p2d.z, len});
        }
        // Vertical corner outlines
        float verticalLen = std::abs(distance);
        for (unsigned int v_idx : boundaryVertexIndices) {
            glm::vec3 p(mesh.vertices[v_idx*3], mesh.vertices[v_idx*3+1], mesh.vertices[v_idx*3+2]);
            glm::vec3 pd = p + offset;
            outlineVertices.insert(outlineVertices.end(), {p.x, p.y, p.z, 0.0f});
            outlineVertices.insert(outlineVertices.end(), {pd.x, pd.y, pd.z, verticalLen});
        }

        // 4. Update GPU buffers
        previewVertexCount = solidVertices.size() / 6;
        if (previewVertexCount > 0) {
            glBindBuffer(GL_ARRAY_BUFFER, previewVBO);
            glBufferData(GL_ARRAY_BUFFER, solidVertices.size() * sizeof(float), solidVertices.data(), GL_DYNAMIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
        }
        previewOutlineVertexCount = outlineVertices.size() / 4;
        if (previewOutlineVertexCount > 0) {
            glBindBuffer(GL_ARRAY_BUFFER, previewOutlineVBO);
            glBufferData(GL_ARRAY_BUFFER, outlineVertices.size() * sizeof(float), outlineVertices.data(), GL_DYNAMIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
        }
    }

    void Renderer::UpdatePreviewLine(const glm::vec3& start, const glm::vec3& end, bool enabled) {
        previewLineEnabled = enabled;
        if (!enabled || previewLineVBO == 0) {
            return;
        }

        float lineData[] = {
            start.x, start.y, start.z, userLineColor.r, userLineColor.g, userLineColor.b, userLineColor.a,
            end.x,   end.y,   end.z,   userLineColor.r, userLineColor.g, userLineColor.b, userLineColor.a
        };

        glBindBuffer(GL_ARRAY_BUFFER, previewLineVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(lineData), lineData, GL_DYNAMIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    void Renderer::UpdateVRPointer(const glm::vec3& start, const glm::vec3& end, bool enabled) {
        vrPointerEnabled = enabled;
        if (!enabled || vrPointerVBO == 0) {
            return;
        }
        glm::vec4 pointerColor = glm::vec4(1.0f, 1.0f, 0.0f, 1.0f);
        float lineData[] = {
            start.x, start.y, start.z, pointerColor.r, pointerColor.g, pointerColor.b, pointerColor.a,
            end.x,   end.y,   end.z,   pointerColor.r, pointerColor.g, pointerColor.b, pointerColor.a
        };
        glBindBuffer(GL_ARRAY_BUFFER, vrPointerVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(lineData), lineData, GL_DYNAMIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    void Renderer::UpdateAxesVBO(const glm::vec4& colorX, const glm::vec4& colorY, const glm::vec4& colorZ, const glm::vec4& posFadeColor, const glm::vec4& negFadeColor) {
        std::vector<float> verticesX = GenerateSingleAxisVertices(axisLength, glm::vec3(1, 0, 0), colorX, posFadeColor, negFadeColor);
        std::vector<float> verticesY = GenerateSingleAxisVertices(axisLength, glm::vec3(0, 1, 0), colorY, posFadeColor, negFadeColor);
        std::vector<float> verticesZ = GenerateSingleAxisVertices(axisLength, glm::vec3(0, 0, 1), colorZ, posFadeColor, negFadeColor);
        
        // Each axis now has 4 vertices (2 for positive, 2 for negative)
        axisVertexCountX = 4; 
        axisVertexCountY = 4;
        axisVertexCountZ = 4;

        std::vector<float> allAxesVertices;
        allAxesVertices.insert(allAxesVertices.end(), verticesX.begin(), verticesX.end());
        allAxesVertices.insert(allAxesVertices.end(), verticesY.begin(), verticesY.end());
        allAxesVertices.insert(allAxesVertices.end(), verticesZ.begin(), verticesZ.end());

        glBindBuffer(GL_ARRAY_BUFFER, axesVBO);
        glBufferData(GL_ARRAY_BUFFER, allAxesVertices.size() * sizeof(float), allAxesVertices.data(), GL_DYNAMIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    void Renderer::RenderSelectionBox(const glm::vec2& start, const glm::vec2& end, int screenWidth, int screenHeight) {
        if (selectionBoxShaderProgram == 0 || selectionBoxVAO == 0) return;
        
        // Convert screen coordinates to Normalized Device Coordinates (NDC) [-1, 1]
        float x1 = (start.x / screenWidth) * 2.0f - 1.0f;
        float y1 = 1.0f - (start.y / screenHeight) * 2.0f;
        float x2 = (end.x / screenWidth) * 2.0f - 1.0f;
        float y2 = 1.0f - (end.y / screenHeight) * 2.0f;

        float vertices[] = {
            x1, y1, // Top-left
            x2, y1, // Top-right
            x2, y2, // Bottom-right
            x1, y2  // Bottom-left
        };

        glBindBuffer(GL_ARRAY_BUFFER, selectionBoxVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        glUseProgram(selectionBoxShaderProgram);
        // Orange color for the box
        glUniform4f(glGetUniformLocation(selectionBoxShaderProgram, "u_Color"), 1.0f, 0.65f, 0.0f, 1.0f);
        
        glDisable(GL_DEPTH_TEST);
        
        // Use stippling for dashed lines
        glEnable(GL_LINE_STIPPLE);
        glLineStipple(4, 0xAAAA); // factor=4, pattern=1010101010101010

        glBindVertexArray(selectionBoxVAO);
        glDrawArrays(GL_LINE_LOOP, 0, 4);
        glBindVertexArray(0);

        glDisable(GL_LINE_STIPPLE);
        glEnable(GL_DEPTH_TEST);
    }

    bool Renderer::CreateSelectionBoxResources() {
        glGenVertexArrays(1, &selectionBoxVAO);
        glGenBuffers(1, &selectionBoxVBO);
        glBindVertexArray(selectionBoxVAO);
        glBindBuffer(GL_ARRAY_BUFFER, selectionBoxVBO);
        // Buffer for 4 vertices (a rectangle), 2 floats each (x, y)
        glBufferData(GL_ARRAY_BUFFER, 4 * 2 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glBindVertexArray(0);
        std::cout << "Renderer: Selection Box VAO/VBO created." << std::endl;
        return selectionBoxVAO != 0 && selectionBoxVBO != 0;
    }

    void Renderer::RenderVRMenuWidget(
        const glm::mat4& view, const glm::mat4& projection,
        const glm::mat4& model,
        const glm::vec3& baseColor, float aberration, float globalAlpha
    ) {
        if (vrMenuWidgetShaderProgram == 0 || splatVAO == 0) return;

        glDepthMask(GL_FALSE);
        glUseProgram(vrMenuWidgetShaderProgram);
        glUniformMatrix4fv(glGetUniformLocation(vrMenuWidgetShaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(vrMenuWidgetShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
        glUniformMatrix4fv(glGetUniformLocation(vrMenuWidgetShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
        
        glUniform3fv(glGetUniformLocation(vrMenuWidgetShaderProgram, "u_baseColor"), 1, glm::value_ptr(baseColor));
        glUniform1f(glGetUniformLocation(vrMenuWidgetShaderProgram, "u_globalAlpha"), globalAlpha);
        glUniform1f(glGetUniformLocation(vrMenuWidgetShaderProgram, "u_aberrationAmount"), aberration);
        
        glBindVertexArray(splatVAO);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
        glDepthMask(GL_TRUE);
    }

    bool Renderer::CreateGhostMeshResources() {
        glGenVertexArrays(1, &ghostMeshVAO);
        glGenBuffers(1, &ghostMeshVBO_vertices);
        glGenBuffers(1, &ghostMeshVBO_normals);
        glGenBuffers(1, &ghostMeshEBO_triangles); // <-- ПЕРЕИМЕНОВАТЬ
        glGenBuffers(1, &ghostMeshEBO_lines);     // <-- ДОБАВИТЬ

        glBindVertexArray(ghostMeshVAO);
        
        glBindBuffer(GL_ARRAY_BUFFER, ghostMeshVBO_vertices);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        
        glBindBuffer(GL_ARRAY_BUFFER, ghostMeshVBO_normals);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(1);
        
        // EBOs are bound during drawing, not here.
        
        glBindVertexArray(0);
        std::cout << "Renderer: Ghost Mesh VAO/VBO/EBOs created." << std::endl;
        return ghostMeshVAO != 0 && ghostMeshVBO_vertices != 0 && ghostMeshVBO_normals != 0 && ghostMeshEBO_triangles != 0 && ghostMeshEBO_lines != 0;
    }

    void Renderer::UpdateGhostMesh(const CadKernel::MeshBuffers& mesh, const std::vector<unsigned int>& wireframeIndices) {
        if (ghostMeshVAO == 0 || mesh.isEmpty()) {
            ClearGhostMesh();
            return;
        }

        // Update vertices
        glBindBuffer(GL_ARRAY_BUFFER, ghostMeshVBO_vertices);
        glBufferData(GL_ARRAY_BUFFER, mesh.vertices.size() * sizeof(float), mesh.vertices.data(), GL_DYNAMIC_DRAW);

        // Update normals
        glBindBuffer(GL_ARRAY_BUFFER, ghostMeshVBO_normals);
        glBufferData(GL_ARRAY_BUFFER, mesh.normals.size() * sizeof(float), mesh.normals.data(), GL_DYNAMIC_DRAW);

        // Update triangle indices
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ghostMeshEBO_triangles);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh.indices.size() * sizeof(unsigned int), mesh.indices.data(), GL_DYNAMIC_DRAW);
        ghostMeshTriangleIndexCount = static_cast<GLsizei>(mesh.indices.size());

        // Update line indices
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ghostMeshEBO_lines);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, wireframeIndices.size() * sizeof(unsigned int), wireframeIndices.data(), GL_DYNAMIC_DRAW);
        ghostMeshLineIndexCount = static_cast<GLsizei>(wireframeIndices.size());

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }

    void Renderer::ClearGhostMesh() {
        if (ghostMeshVAO != 0) {
            ghostMeshTriangleIndexCount = 0;
            ghostMeshLineIndexCount = 0; // <-- ДОБАВИТЬ
        }
    }

    void Renderer::RenderSingleObject(
        const glm::mat4& view, const glm::mat4& projection,
        Urbaxio::Engine::SceneObject* obj,
        const glm::mat4& modelMatrix,
        const glm::vec3& color,
        bool unlit
    ) {
        if (!obj || obj->vao == 0 || obj->index_count == 0 || objectShaderProgram == 0) {
            return;
        }

        glUseProgram(objectShaderProgram);
        glUniformMatrix4fv(glGetUniformLocation(objectShaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(objectShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
        glUniformMatrix4fv(glGetUniformLocation(objectShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(modelMatrix));
        glUniform3fv(glGetUniformLocation(objectShaderProgram, "objectColor"), 1, glm::value_ptr(color));
        glUniform1i(glGetUniformLocation(objectShaderProgram, "u_unlit"), unlit ? 1 : 0);
        glUniform1f(glGetUniformLocation(objectShaderProgram, "overrideAlpha"), 1.0f);
        
        if (!unlit) {
            glm::vec3 viewPos = glm::vec3(glm::inverse(view)[3]);
            glm::vec3 lightDir = -glm::normalize(glm::vec3(glm::inverse(view)[2]));
            glUniform3fv(glGetUniformLocation(objectShaderProgram, "lightDir"), 1, glm::value_ptr(lightDir));
            glUniform3fv(glGetUniformLocation(objectShaderProgram, "lightColor"), 1, glm::value_ptr(glm::vec3(1.0f)));
            glUniform1f(glGetUniformLocation(objectShaderProgram, "ambientStrength"), 0.3f);
            glUniform3fv(glGetUniformLocation(objectShaderProgram, "viewPos"), 1, glm::value_ptr(viewPos));
        }

        glBindVertexArray(obj->vao);
        glDrawElements(GL_TRIANGLES, obj->index_count, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }

} // namespace Urbaxio