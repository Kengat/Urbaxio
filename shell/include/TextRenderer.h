#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <string>
#include <vector>
#include <map>

namespace Urbaxio {

struct Glyph {
    int unicode = 0;
    float advance = 0.0f;
    glm::vec4 planeBounds{0.0f};
    glm::vec4 atlasBounds{0.0f};
};

class TextRenderer {
public:
    TextRenderer();
    ~TextRenderer();

    bool Initialize(const std::string& fontJsonPath, const std::string& atlasImagePath);
    void Shutdown();

    void AddText(const std::string& text,
                 const glm::vec3& position,
                 const glm::vec4& color,
                 float scale,
                 const glm::mat4& viewMatrix,
                 bool billboard = true);

    void AddTextOnPanel(const std::string& text,
                      const glm::vec3& localPosition,
                      const glm::vec4& color,
                      float height);
    void ClearPanelModelMatrix();
    void SetPanelModelMatrix(const glm::mat4& modelMatrix);

    void Render(const glm::mat4& view, const glm::mat4& projection);

private:
    struct Vertex {
        glm::vec3 pos;
        glm::vec2 uv;
        glm::vec4 color;
    };

    struct PanelVertex {
        glm::vec3 pos; // Position relative to panel origin
        glm::vec2 uv;
        glm::vec4 color;
    };

    bool CreateShaderProgram();
    bool LoadFont(const std::string& fontJsonPath, const std::string& atlasImagePath);

    GLuint shaderProgram_ = 0;
    GLuint vao_ = 0;
    GLuint vbo_ = 0;
    GLuint fontAtlasTexture_ = 0;

    GLuint panel_shaderProgram_ = 0;
    GLuint panel_vao_ = 0;
    GLuint panel_vbo_ = 0;

    float atlasWidth_ = 1.0f;
    float atlasHeight_ = 1.0f;
    float pxRange_ = 4.0f;
    float fontScale_ = 32.0f;
    float lineHeight_ = 1.0f;

    std::map<int, Glyph> glyphs_;
    std::vector<Vertex> vertices_;
    std::vector<PanelVertex> panelVertices_;
    glm::mat4 currentPanelModelMatrix_ = glm::mat4(1.0f);
};

} // namespace Urbaxio

