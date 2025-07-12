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
            "void main() {\n"
            "    vec3 norm = normalize(NormalWorld);\n"
            "    vec3 ambient = ambientStrength * lightColor * objectColor;\n"
            "    float diff = max(dot(norm, normalize(lightDir)), 0.0);\n"
            "    vec3 diffuse = diff * lightColor * objectColor;\n"
            "    vec3 result = ambient + diffuse;\n"
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
    }
    Renderer::~Renderer() { Cleanup(); }
    bool Renderer::Initialize() { std::cout << "Renderer: Initializing..." << std::endl; GLfloat range[2] = { 1.0f, 1.0f }; glGetFloatv(GL_ALIASED_LINE_WIDTH_RANGE, range); maxLineWidth = std::max(1.0f, range[1]); std::cout << "Renderer: Supported ALIASED Line Width Range: [" << range[0] << ", " << maxLineWidth << "]" << std::endl; if (!CreateShaderPrograms()) return false; if (!CreateGridResources()) return false; if (!CreateAxesResources()) return false; if (!CreateSplatResources()) return false; if (!CreateUserLinesResources()) return false; if (!CreateMarkerResources()) return false; if (!CreatePreviewResources()) return false; glEnable(GL_DEPTH_TEST); glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); std::cout << "Renderer: Initialization successful." << std::endl; return true; }
    void Renderer::SetViewport(int x, int y, int width, int height) { /* ... same ... */ if (width > 0 && height > 0) { glViewport(x, y, width, height); } }
    
    void Renderer::RenderFrame(
        SDL_Window* window,
        const Urbaxio::Camera& camera,
        Urbaxio::Engine::Scene* scene,
        // Appearance
        const glm::vec3& defaultObjectColor,
        const glm::vec3& lightDir, const glm::vec3& lightColor, float ambientStrength,
        bool showGrid, bool showAxes, float axisLineWidth, float negAxisLineWidth,
        const glm::vec3& gridColor, const glm::vec4& axisColorX, const glm::vec4& axisColorY, const glm::vec4& axisColorZ,
        const glm::vec4& positiveAxisFadeColor, const glm::vec4& negativeAxisFadeColor,
        // Test Elements
        const glm::vec4& splatColor, float splatBlurStrength,
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
        bool isDrawingActive, const glm::vec3& rubberBandStart, const glm::vec3& rubberBandEnd,
        const SnapResult& currentSnap,
        ImDrawData* imguiDrawData
    ) {
        int display_w, display_h; SDL_GetWindowSize(window, &display_w, &display_h); if (display_w <= 0 || display_h <= 0) return; glm::mat4 view = camera.GetViewMatrix(); glm::mat4 projection = camera.GetProjectionMatrix((float)display_w / (float)display_h); glm::mat4 identityModel = glm::mat4(1.0f);
        
        // Common calculations for dynamic effects
        float distanceToCamera = glm::length(camera.Position);
        const float referenceDistance = 30.0f;
        float distanceScale = distanceToCamera / referenceDistance;
        
        // --- DRAW GRID (with new shader) ---
        if (showGrid && gridVAO != 0 && gridShaderProgram != 0) {
            glLineWidth(1.0f); // Keep grid lines thin
            glUseProgram(gridShaderProgram);
            glUniformMatrix4fv(glGetUniformLocation(gridShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(identityModel));
            glUniformMatrix4fv(glGetUniformLocation(gridShaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
            glUniformMatrix4fv(glGetUniformLocation(gridShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
            glUniform3fv(glGetUniformLocation(gridShaderProgram, "u_gridColor"), 1, glm::value_ptr(gridColor));
            glUniform3fv(glGetUniformLocation(gridShaderProgram, "u_cursorWorldPos"), 1, glm::value_ptr(cursorWorldPos));
            glUniform1f(glGetUniformLocation(gridShaderProgram, "u_cursorRadius"), cursorRadius);
            glUniform1f(glGetUniformLocation(gridShaderProgram, "u_intensity"), intensity);
            // Dynamic hole effect around center
            const float baseHoleStart = 0.25f;
            const float baseHoleEnd = 0.75f;
            glUniform1f(glGetUniformLocation(gridShaderProgram, "u_holeStart"), baseHoleStart * distanceScale);
            glUniform1f(glGetUniformLocation(gridShaderProgram, "u_holeEnd"), baseHoleEnd * distanceScale);
            glBindVertexArray(gridVAO);
            glDrawArrays(GL_LINES, 0, gridVertexCount);
            glBindVertexArray(0);
        }
        // --- DRAW USER LINES (can use simple shader) ---
        if (userLinesVAO != 0 && simpleLineShaderProgram != 0 && userLinesVertexCount > 0) {
            glLineWidth(2.0f);
            glUseProgram(simpleLineShaderProgram);
            glUniformMatrix4fv(glGetUniformLocation(simpleLineShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(identityModel));
            glUniformMatrix4fv(glGetUniformLocation(simpleLineShaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
            glUniformMatrix4fv(glGetUniformLocation(simpleLineShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
            glBindVertexArray(userLinesVAO);
            glDrawArrays(GL_LINES, 0, userLinesVertexCount);
            glBindVertexArray(0);
            glLineWidth(1.0f);
        }
        // --- DRAW RUBBER BAND LINE (using simple shader) ---
        if (isDrawingActive && simpleLineShaderProgram != 0) {
            glLineWidth(1.0f);
            glUseProgram(simpleLineShaderProgram);
            glUniformMatrix4fv(glGetUniformLocation(simpleLineShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(identityModel));
            glUniformMatrix4fv(glGetUniformLocation(simpleLineShaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
            glUniformMatrix4fv(glGetUniformLocation(simpleLineShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
            GLuint tempVBO, tempVAO;
            float lineData[] = {
                rubberBandStart.x, rubberBandStart.y, rubberBandStart.z, userLineColor.r, userLineColor.g, userLineColor.b, userLineColor.a,
                rubberBandEnd.x,   rubberBandEnd.y,   rubberBandEnd.z,   userLineColor.r, userLineColor.g, userLineColor.b, userLineColor.a
            };
            glGenVertexArrays(1, &tempVAO);
            glGenBuffers(1, &tempVBO);
            glBindVertexArray(tempVAO);
            glBindBuffer(GL_ARRAY_BUFFER, tempVBO);
            glBufferData(GL_ARRAY_BUFFER, sizeof(lineData), lineData, GL_DYNAMIC_DRAW);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)0);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)(3 * sizeof(float)));
            glEnableVertexAttribArray(1);
            glBindVertexArray(tempVAO);
            glDrawArrays(GL_LINES, 0, 2);
            glBindVertexArray(0);
            glDeleteBuffers(1, &tempVBO);
            glDeleteVertexArrays(1, &tempVAO);
        }
        if (objectShaderProgram != 0 && scene) { glLineWidth(1.0f); glUseProgram(objectShaderProgram); glUniformMatrix4fv(glGetUniformLocation(objectShaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view)); glUniformMatrix4fv(glGetUniformLocation(objectShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection)); glUniform3fv(glGetUniformLocation(objectShaderProgram, "lightDir"), 1, glm::value_ptr(lightDir)); glUniform3fv(glGetUniformLocation(objectShaderProgram, "lightColor"), 1, glm::value_ptr(lightColor)); glUniform1f(glGetUniformLocation(objectShaderProgram, "ambientStrength"), ambientStrength); glUniform3fv(glGetUniformLocation(objectShaderProgram, "viewPos"), 1, glm::value_ptr(camera.Position)); glUniform1f(glGetUniformLocation(objectShaderProgram, "overrideAlpha"), 1.0f);
            
            // 1. Draw lit objects (excluding all marker templates)
            for (const auto* obj : scene->get_all_objects()) {
                if (obj && obj->vao != 0 && obj->index_count > 0 && 
                    obj->get_name() != "CenterMarker" && 
                    obj->get_name() != "UnitSphereMarker" && 
                    obj->get_name() != "UnitCapsuleMarker") { // Skip all marker templates
                    glUniform3fv(glGetUniformLocation(objectShaderProgram, "objectColor"), 1, glm::value_ptr(defaultObjectColor));
                    glUniformMatrix4fv(glGetUniformLocation(objectShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(identityModel));
                    glBindVertexArray(obj->vao);
                    glDrawElements(GL_TRIANGLES, obj->index_count, GL_UNSIGNED_INT, 0);
                    glBindVertexArray(0);
                }
            }

            // 2. Draw hover highlight (if any, and not the same as selected)
            if (hoveredObjId != 0 && !hoveredFaceTriangleIndices.empty()) {
                bool isSameAsSelected = (hoveredObjId == selectedObjId) && (hoveredFaceTriangleIndices == selectedTriangleIndices);
                if (!isSameAsSelected) {
                    Urbaxio::Engine::SceneObject* hoveredObj = scene->get_object_by_id(hoveredObjId);
                     if (hoveredObj && hoveredObj->vao != 0) {
                        glUniform3fv(glGetUniformLocation(objectShaderProgram, "objectColor"), 1, glm::value_ptr(hoverHighlightColor));
                        glEnable(GL_POLYGON_OFFSET_FILL);
                        glPolygonOffset(-2.0f, -2.0f); // Use a different offset
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
            
            // 3. Draw selection highlight (on top of everything)
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
            
            // 4. Draw Push/Pull Preview
            if (previewVertexCount > 0) {
                glDepthMask(GL_FALSE); // Disable writing to depth buffer for transparency
                glUniform3fv(glGetUniformLocation(objectShaderProgram, "objectColor"), 1, glm::value_ptr(hoverHighlightColor));
                glUniform1f(glGetUniformLocation(objectShaderProgram, "overrideAlpha"), 0.5f);
                glBindVertexArray(previewVAO);
                glDrawArrays(GL_TRIANGLES, 0, previewVertexCount);
                glBindVertexArray(0);
                glDepthMask(GL_TRUE); // Re-enable depth writing
                glUniform1f(glGetUniformLocation(objectShaderProgram, "overrideAlpha"), 1.0f);
            }
        }

        // --- DRAW UNLIT OBJECTS (CENTER SPHERE & MARKERS) ---
        if (unlitShaderProgram != 0 && scene) {
            glUseProgram(unlitShaderProgram);
            glUniformMatrix4fv(glGetUniformLocation(unlitShaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
            glUniformMatrix4fv(glGetUniformLocation(unlitShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
            glUniform3fv(glGetUniformLocation(unlitShaderProgram, "u_cursorWorldPos"), 1, glm::value_ptr(cursorWorldPos));
            glUniform1f(glGetUniformLocation(unlitShaderProgram, "u_cursorRadius"), cursorRadius);
            glUniform1f(glGetUniformLocation(unlitShaderProgram, "u_intensity"), intensity);
            
            // Find marker templates
            Urbaxio::Engine::SceneObject* center_marker = nullptr;
            Urbaxio::Engine::SceneObject* sphere_marker_template = nullptr;
            Urbaxio::Engine::SceneObject* capsule_marker_template = nullptr;
            
            for(auto* obj : scene->get_all_objects()){
                if(obj && obj->get_name() == "CenterMarker") {
                    center_marker = obj;
                } else if(obj && obj->get_name() == "UnitSphereMarker") {
                    sphere_marker_template = obj;
                } else if(obj && obj->get_name() == "UnitCapsuleMarker") {
                    capsule_marker_template = obj;
                }
            }

            // Draw Center Marker (always white, no interactivity)
            if (center_marker && center_marker->vao != 0) {
                 float scale = (distanceToCamera / referenceDistance) * 0.7f;
                 // Remove the minimum size clamp
                 // scale = glm::max(scale, 0.1f); 
                 glUniform1f(glGetUniformLocation(unlitShaderProgram, "u_fadeStart"), 0.0); // No gradient for center
                 glUniform1f(glGetUniformLocation(unlitShaderProgram, "u_fadeEnd"), 0.0);   // No gradient for center
                 glm::mat4 modelMatrix = glm::scale(glm::mat4(1.0f), glm::vec3(scale));
                 glUniformMatrix4fv(glGetUniformLocation(unlitShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(modelMatrix));
                 glUniform3f(glGetUniformLocation(unlitShaderProgram, "u_baseColor"), 1.0f, 1.0f, 1.0f);
                 glUniform3f(glGetUniformLocation(unlitShaderProgram, "u_fadeColor"), 1.0f, 1.0f, 1.0f);
                 glBindVertexArray(center_marker->vao);
                 glDrawElements(GL_TRIANGLES, center_marker->index_count, GL_UNSIGNED_INT, 0);
            }

            // Draw Axis Markers
            if (showAxes && sphere_marker_template && sphere_marker_template->vao != 0 && capsule_marker_template && capsule_marker_template->vao != 0) {
                const float markerBaseScale = 0.15f; // Smaller than center sphere
                const int maxMarkerDist = 100;
                const float maxMarkerWorldSize = 0.5f;

                // Pass gradient uniforms for markers
                const float baseFadeStart = 0.5f;
                const float baseFadeEnd = 4.0f;
                glUniform1f(glGetUniformLocation(unlitShaderProgram, "u_fadeStart"), baseFadeStart * distanceScale);
                glUniform1f(glGetUniformLocation(unlitShaderProgram, "u_fadeEnd"), baseFadeEnd * distanceScale);

                struct AxisInfo { glm::vec3 dir; glm::vec4 color; };
                std::vector<AxisInfo> axes_info = {
                    {glm::vec3(1,0,0), axisColorX},
                    {glm::vec3(0,1,0), axisColorY},
                    {glm::vec3(0,0,1), axisColorZ}
                };
                
                // Set fade color for all markers
                glUniform3fv(glGetUniformLocation(unlitShaderProgram, "u_fadeColor"), 1, glm::value_ptr(glm::vec3(positiveAxisFadeColor)));

                for(const auto& axis : axes_info) {
                    glUniform3fv(glGetUniformLocation(unlitShaderProgram, "u_baseColor"), 1, glm::value_ptr(glm::vec3(axis.color)));
                    for(int i = 5; i <= maxMarkerDist; i += 5) { // Start from 5 to exclude origin
                        glm::vec3 position = axis.dir * (float)i;
                        float distToMarker = glm::length(camera.Position - position);
                        float scale = (distToMarker / referenceDistance) * markerBaseScale;

                        // Clamp max size
                        if (scale > maxMarkerWorldSize) {
                            scale = maxMarkerWorldSize;
                        }

                        glm::mat4 translationMatrix = glm::translate(glm::mat4(1.0f), position);
                        glm::mat4 scaleMatrix = glm::scale(glm::mat4(1.0f), glm::vec3(scale));
                        
                        if (i % 10 == 0) { // Capsule at 10, 20, 30...
                            glm::quat rotation;
                            // Check if it's the Z-axis
                            if (glm::abs(axis.dir.z) > 0.99) { // It's the Z axis
                                // For Z-axis: first lay flat on XY plane, then rotate 45 degrees around Z
                                rotation = glm::angleAxis(glm::radians(90.0f), glm::vec3(1,0,0));
                                rotation = glm::angleAxis(glm::radians(45.0f), glm::vec3(0,0,1)) * rotation;
                            } else { // X or Y axis
                                // For X/Y axes: lay flat on XY plane, then point along axis
                                rotation = glm::angleAxis(glm::radians(90.0f), glm::vec3(1,0,0));
                                rotation = glm::angleAxis(glm::atan(axis.dir.y, axis.dir.x), glm::vec3(0,0,1)) * rotation;
                            }
                            
                            glm::mat4 rotationMatrix = glm::mat4_cast(rotation);
                            glm::mat4 modelMatrix = translationMatrix * rotationMatrix * scaleMatrix;
                            glUniformMatrix4fv(glGetUniformLocation(unlitShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(modelMatrix));
                            glBindVertexArray(capsule_marker_template->vao);
                            glDrawElements(GL_TRIANGLES, capsule_marker_template->index_count, GL_UNSIGNED_INT, 0);
                        } else { // Sphere at 5, 15, 25...
                            glm::mat4 modelMatrix = translationMatrix * scaleMatrix;
                            glUniformMatrix4fv(glGetUniformLocation(unlitShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(modelMatrix));
                            glBindVertexArray(sphere_marker_template->vao);
                            glDrawElements(GL_TRIANGLES, sphere_marker_template->index_count, GL_UNSIGNED_INT, 0);
                        }
                    }
                }
            }
            glBindVertexArray(0);
        }

        // --- DRAW AXES (with new shader) ---
        if (showAxes && axesVAO != 0 && axisShaderProgram != 0) {
            UpdateAxesVBO(axisColorX, axisColorY, axisColorZ, positiveAxisFadeColor, negativeAxisFadeColor);
            
            glUseProgram(axisShaderProgram);
            glUniformMatrix4fv(glGetUniformLocation(axisShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(identityModel));
            glUniformMatrix4fv(glGetUniformLocation(axisShaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
            glUniformMatrix4fv(glGetUniformLocation(axisShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
            glUniform3fv(glGetUniformLocation(axisShaderProgram, "u_cursorWorldPos"), 1, glm::value_ptr(cursorWorldPos));
            glUniform1f(glGetUniformLocation(axisShaderProgram, "u_cursorRadius"), cursorRadius);
            glUniform1f(glGetUniformLocation(axisShaderProgram, "u_intensity"), intensity);

            // Dynamic fade distances for axis gradient
            const float baseFadeStart = 0.5f;
            const float baseFadeEnd = 4.0f;
            glUniform1f(glGetUniformLocation(axisShaderProgram, "u_fadeStart"), baseFadeStart * distanceScale);
            glUniform1f(glGetUniformLocation(axisShaderProgram, "u_fadeEnd"), baseFadeEnd * distanceScale);
            
            // Dynamic hole effect around center
            const float baseHoleStart = 0.25f;
            const float baseHoleEnd = 0.75f;
            glUniform1f(glGetUniformLocation(axisShaderProgram, "u_holeStart"), baseHoleStart * distanceScale);
            glUniform1f(glGetUniformLocation(axisShaderProgram, "u_holeEnd"), baseHoleEnd * distanceScale);
            
            // Pass fade colors to shader
            glUniform4fv(glGetUniformLocation(axisShaderProgram, "u_positiveFadeColor"), 1, glm::value_ptr(positiveAxisFadeColor));
            glUniform4fv(glGetUniformLocation(axisShaderProgram, "u_negativeFadeColor"), 1, glm::value_ptr(negativeAxisFadeColor));
            
            glBindVertexArray(axesVAO);
            
            // Draw negative axes first with thinner lines
            glLineWidth(negAxisLineWidth);
            glDrawArrays(GL_LINES, 2, 2); // X neg
            glDrawArrays(GL_LINES, 6, 2); // Y neg
            glDrawArrays(GL_LINES, 10, 2); // Z neg
            
            // Draw positive axes on top with thicker lines
            glLineWidth(axisLineWidth);
            glDrawArrays(GL_LINES, 0, 2); // X pos
            glDrawArrays(GL_LINES, 4, 2); // Y pos
            glDrawArrays(GL_LINES, 8, 2); // Z pos

            glBindVertexArray(0);
        }
        if (splatShaderProgram != 0 && splatVAO != 0) { glLineWidth(1.0f); glUseProgram(splatShaderProgram); glUniform4fv(glGetUniformLocation(splatShaderProgram, "splatColor"), 1, glm::value_ptr(splatColor)); glUniform1f(glGetUniformLocation(splatShaderProgram, "blurStrength"), splatBlurStrength); glUniformMatrix4fv(glGetUniformLocation(splatShaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view)); glUniformMatrix4fv(glGetUniformLocation(splatShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection)); glBindVertexArray(splatVAO); glm::mat4 modelStatic = glm::translate(glm::mat4(1.0f), splatPosStatic); glUniformMatrix4fv(glGetUniformLocation(splatShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(modelStatic)); glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0); glm::mat4 modelBillboard = glm::translate(glm::mat4(1.0f), splatPosBillboard); glm::mat3 viewRot = glm::mat3(view); glm::mat3 counterRot = glm::transpose(viewRot); modelBillboard = modelBillboard * glm::mat4(counterRot); glUniformMatrix4fv(glGetUniformLocation(splatShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(modelBillboard)); glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0); glBindVertexArray(0); }
        glDisable(GL_DEPTH_TEST); DrawSnapMarker(currentSnap, camera, view, projection, display_w, display_h); glEnable(GL_DEPTH_TEST);
        glLineWidth(1.0f); glUseProgram(0); ImGui_ImplOpenGL3_RenderDrawData(imguiDrawData);
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
    bool Renderer::CreateUserLinesResources() { /* ... same ... */ glGenVertexArrays(1, &userLinesVAO); glGenBuffers(1, &userLinesVBO); glBindVertexArray(userLinesVAO); glBindBuffer(GL_ARRAY_BUFFER, userLinesVBO); const size_t maxLinePoints = 400; glBufferData(GL_ARRAY_BUFFER, maxLinePoints * 7 * sizeof(float), nullptr, GL_DYNAMIC_DRAW); glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)0); glEnableVertexAttribArray(0); glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)(3 * sizeof(float))); glEnableVertexAttribArray(1); glBindBuffer(GL_ARRAY_BUFFER, 0); glBindVertexArray(0); std::cout << "Renderer: User Lines VAO/VBO created." << std::endl; return userLinesVAO != 0 && userLinesVBO != 0; }
    void Renderer::UpdateUserLinesBuffer(const std::map<uint64_t, Engine::Line>& lines, const std::set<uint64_t>& selectedLineIDs) {
        if (userLinesVBO == 0) {
            userLinesVertexCount = 0;
            return;
        }
        
        if (lines.empty()) {
            userLinesVertexCount = 0;
            return;
        }
        
        std::vector<float> lineData;
        lineData.reserve(lines.size() * 2 * 7);
        
        for (const auto& [lineID, line] : lines) {
            glm::vec4 currentColor = userLineColor;
            if (selectedLineIDs.find(lineID) != selectedLineIDs.end()) {
                currentColor = selectedUserLineColor;
            }
            
            // Start point
            lineData.push_back(line.start.x);
            lineData.push_back(line.start.y);
            lineData.push_back(line.start.z);
            lineData.push_back(currentColor.r);
            lineData.push_back(currentColor.g);
            lineData.push_back(currentColor.b);
            lineData.push_back(currentColor.a);
            
            // End point
            lineData.push_back(line.end.x);
            lineData.push_back(line.end.y);
            lineData.push_back(line.end.z);
            lineData.push_back(currentColor.r);
            lineData.push_back(currentColor.g);
            lineData.push_back(currentColor.b);
            lineData.push_back(currentColor.a);
        }
        
        glBindBuffer(GL_ARRAY_BUFFER, userLinesVBO);
        glBufferData(GL_ARRAY_BUFFER, lineData.size() * sizeof(float), lineData.data(), GL_DYNAMIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        userLinesVertexCount = static_cast<int>(lineData.size() / 7);
    }
    bool Renderer::CreateMarkerResources() { /* ... same ... */ std::vector<float> circleVertices = GenerateCircleVertices(24); if (circleVertices.empty()) return false; markerVertexCounts[MarkerShape::CIRCLE] = static_cast<int>(circleVertices.size() / 2); glGenVertexArrays(1, &markerVAOs[MarkerShape::CIRCLE]); glGenBuffers(1, &markerVBOs[MarkerShape::CIRCLE]); glBindVertexArray(markerVAOs[MarkerShape::CIRCLE]); glBindBuffer(GL_ARRAY_BUFFER, markerVBOs[MarkerShape::CIRCLE]); glBufferData(GL_ARRAY_BUFFER, circleVertices.size() * sizeof(float), circleVertices.data(), GL_STATIC_DRAW); glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0); glEnableVertexAttribArray(0); glBindVertexArray(0); std::cout << "Renderer: Circle Marker VAO/VBO created (" << markerVertexCounts[MarkerShape::CIRCLE] << " vertices)." << std::endl; std::vector<float> diamondVertices = GenerateDiamondVertices(); if (diamondVertices.empty()) return false; markerVertexCounts[MarkerShape::DIAMOND] = static_cast<int>(diamondVertices.size() / 2); glGenVertexArrays(1, &markerVAOs[MarkerShape::DIAMOND]); glGenBuffers(1, &markerVBOs[MarkerShape::DIAMOND]); glBindVertexArray(markerVAOs[MarkerShape::DIAMOND]); glBindBuffer(GL_ARRAY_BUFFER, markerVBOs[MarkerShape::DIAMOND]); glBufferData(GL_ARRAY_BUFFER, diamondVertices.size() * sizeof(float), diamondVertices.data(), GL_STATIC_DRAW); glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0); glEnableVertexAttribArray(0); glBindVertexArray(0); std::cout << "Renderer: Diamond Marker VAO/VBO created (" << markerVertexCounts[MarkerShape::DIAMOND] << " vertices)." << std::endl; return true; }
    void Renderer::DrawSnapMarker(const SnapResult& snap, const Camera& camera, const glm::mat4& view, const glm::mat4& proj, int screenWidth, int screenHeight) { /* ... same ... */ if (!snap.snapped || markerShaderProgram == 0) return; MarkerShape shape = MarkerShape::CIRCLE; glm::vec4 color = snapMarkerColorPoint; float currentMarkerSize = markerScreenSize; switch (snap.type) { case SnapType::ENDPOINT: case SnapType::ORIGIN: shape = MarkerShape::CIRCLE; color = snapMarkerColorPoint; currentMarkerSize = markerScreenSize; break; case SnapType::MIDPOINT: shape = MarkerShape::CIRCLE; color = snapMarkerColorMidpoint; currentMarkerSize = markerScreenSizeMidpoint; break; case SnapType::ON_EDGE: shape = MarkerShape::DIAMOND; color = snapMarkerColorOnEdge; currentMarkerSize = markerScreenSizeOnEdge; break; case SnapType::AXIS_X: shape = MarkerShape::DIAMOND; color = snapMarkerColorAxisX; currentMarkerSize = markerScreenSize; break; case SnapType::AXIS_Y: shape = MarkerShape::DIAMOND; color = snapMarkerColorAxisY; currentMarkerSize = markerScreenSize; break; case SnapType::AXIS_Z: shape = MarkerShape::DIAMOND; color = snapMarkerColorAxisZ; currentMarkerSize = markerScreenSize; break; case SnapType::ON_FACE: shape = MarkerShape::CIRCLE; color = snapMarkerColorOnEdge; /* Using OnEdge magenta for now, define snapMarkerColorOnFace later */ currentMarkerSize = markerScreenSize; break; default: return; } if (markerVAOs.find(shape) == markerVAOs.end()) return; GLuint vao = markerVAOs[shape]; int vertexCount = markerVertexCounts[shape]; if (vao == 0 || vertexCount == 0) return; glUseProgram(markerShaderProgram); glUniform3fv(glGetUniformLocation(markerShaderProgram, "u_WorldPos"), 1, glm::value_ptr(snap.worldPoint)); glUniform1f(glGetUniformLocation(markerShaderProgram, "u_ScreenSize"), currentMarkerSize); glUniformMatrix4fv(glGetUniformLocation(markerShaderProgram, "u_ViewMatrix"), 1, GL_FALSE, glm::value_ptr(view)); glUniformMatrix4fv(glGetUniformLocation(markerShaderProgram, "u_ProjMatrix"), 1, GL_FALSE, glm::value_ptr(proj)); glUniform2f(glGetUniformLocation(markerShaderProgram, "u_ViewportSize"), (float)screenWidth, (float)screenHeight); glUniform4fv(glGetUniformLocation(markerShaderProgram, "u_Color"), 1, glm::value_ptr(color)); glBindVertexArray(vao); if (shape == MarkerShape::CIRCLE) { glDrawArrays(GL_TRIANGLE_FAN, 0, vertexCount); } else if (shape == MarkerShape::DIAMOND) { glLineWidth(2.0f); glDrawArrays(GL_LINE_LOOP, 0, vertexCount -1); } glBindVertexArray(0); glLineWidth(1.0f); }
    void Renderer::Cleanup() {
        std::cout << "Renderer: Cleaning up resources..." << std::endl;
        if (gridVAO != 0) glDeleteVertexArrays(1, &gridVAO); gridVAO = 0; if (gridVBO != 0) glDeleteBuffers(1, &gridVBO); gridVBO = 0;
        if (axesVAO != 0) glDeleteVertexArrays(1, &axesVAO); axesVAO = 0; if (axesVBO != 0) glDeleteBuffers(1, &axesVBO); axesVBO = 0;
        if (splatVAO != 0) glDeleteVertexArrays(1, &splatVAO); splatVAO = 0; if (splatVBO != 0) glDeleteBuffers(1, &splatVBO); splatVBO = 0; if (splatEBO != 0) glDeleteBuffers(1, &splatEBO); splatEBO = 0;
        if (userLinesVAO != 0) glDeleteVertexArrays(1, &userLinesVAO); userLinesVAO = 0; if (userLinesVBO != 0) glDeleteBuffers(1, &userLinesVBO); userLinesVBO = 0;
        if (previewVAO != 0) glDeleteVertexArrays(1, &previewVAO); previewVAO = 0; if (previewVBO != 0) glDeleteBuffers(1, &previewVBO); previewVBO = 0;
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

    void Renderer::UpdatePushPullPreview(const Engine::SceneObject& object, const std::vector<size_t>& faceIndices, const glm::vec3& direction, float distance) {
        if (faceIndices.empty() || std::abs(distance) < 1e-4) {
            previewVertexCount = 0;
            return;
        }

        const auto& mesh = object.get_mesh_buffers();
        if (!object.has_mesh()) {
            previewVertexCount = 0;
            return;
        }
        
        std::vector<float> previewVertices;
        glm::vec3 offset = direction * distance;

        for (size_t baseIndex : faceIndices) {
            unsigned int i0 = mesh.indices[baseIndex];
            unsigned int i1 = mesh.indices[baseIndex + 1];
            unsigned int i2 = mesh.indices[baseIndex + 2];

            glm::vec3 v0(mesh.vertices[i0*3], mesh.vertices[i0*3+1], mesh.vertices[i0*3+2]);
            glm::vec3 v1(mesh.vertices[i1*3], mesh.vertices[i1*3+1], mesh.vertices[i1*3+2]);
            glm::vec3 v2(mesh.vertices[i2*3], mesh.vertices[i2*3+1], mesh.vertices[i2*3+2]);
            
            // --- Top Cap ---
            glm::vec3 v0d = v0 + offset;
            glm::vec3 v1d = v1 + offset;
            glm::vec3 v2d = v2 + offset;
            previewVertices.insert(previewVertices.end(), {v0d.x, v0d.y, v0d.z, direction.x, direction.y, direction.z});
            previewVertices.insert(previewVertices.end(), {v1d.x, v1d.y, v1d.z, direction.x, direction.y, direction.z});
            previewVertices.insert(previewVertices.end(), {v2d.x, v2d.y, v2d.z, direction.x, direction.y, direction.z});
            
            // --- Sides (3 quads) ---
            glm::vec3 p[3] = {v0, v1, v2};
            for(int i=0; i<3; ++i) {
                glm::vec3 p1 = p[i];
                glm::vec3 p2 = p[(i+1)%3];
                glm::vec3 p1d = p1 + offset;
                glm::vec3 p2d = p2 + offset;

                glm::vec3 sideNormal = glm::normalize(glm::cross(p2-p1, direction));
                
                previewVertices.insert(previewVertices.end(), {p1.x, p1.y, p1.z, sideNormal.x, sideNormal.y, sideNormal.z});
                previewVertices.insert(previewVertices.end(), {p2.x, p2.y, p2.z, sideNormal.x, sideNormal.y, sideNormal.z});
                previewVertices.insert(previewVertices.end(), {p1d.x, p1d.y, p1d.z, sideNormal.x, sideNormal.y, sideNormal.z});

                previewVertices.insert(previewVertices.end(), {p1d.x, p1d.y, p1d.z, sideNormal.x, sideNormal.y, sideNormal.z});
                previewVertices.insert(previewVertices.end(), {p2.x, p2.y, p2.z, sideNormal.x, sideNormal.y, sideNormal.z});
                previewVertices.insert(previewVertices.end(), {p2d.x, p2d.y, p2d.z, sideNormal.x, sideNormal.y, sideNormal.z});
            }
        }

        previewVertexCount = previewVertices.size() / 6;
        glBindBuffer(GL_ARRAY_BUFFER, previewVBO);
        glBufferData(GL_ARRAY_BUFFER, previewVertices.size() * sizeof(float), previewVertices.data(), GL_DYNAMIC_DRAW);
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

} // namespace Urbaxio