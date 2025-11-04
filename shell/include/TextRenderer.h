#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <string>
#include <vector>
#include <map>
#include <optional>
#include "ui/IVRWidget.h"



namespace Urbaxio {

enum class TextAlign { LEFT, CENTER };

struct Glyph {
    int unicode = 0;
    float advance = 0.0f;
    glm::vec4 planeBounds{0.0f};
    glm::vec4 atlasBounds{0.0f};
};

// --- Comparator for glm::vec4 values ---
struct Vec4Comparator {
    bool operator()(const glm::vec4& a, const glm::vec4& b) const {
        if (a.x != b.x) return a.x < b.x;
        if (a.y != b.y) return a.y < b.y;
        if (a.z != b.z) return a.z < b.z;
        return a.w < b.w;
    }
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
                      float height,
                      TextAlign alignment = TextAlign::CENTER,
                      const std::optional<UI::MaskData>& mask = std::nullopt);
    void ClearPanelModelMatrix();
    void SetPanelModelMatrix(const glm::mat4& modelMatrix);

    void Render(const glm::mat4& view, const glm::mat4& projection);
    void RenderPanelText(const glm::mat4& view, const glm::mat4& projection);

    glm::vec2 GetTextSize(const std::string& text, float height) const;

private:
    struct Vertex {
        glm::vec3 pos;
        glm::vec2 uv;
        glm::vec4 color;
    };

    struct PanelVertex {
        glm::vec3 pos;
        glm::vec2 uv;
        glm::vec4 color;
    };

    using MaskInfo = std::optional<UI::MaskData>;

    // New batching key that accounts for mask and model matrix
    struct PanelKey {
        MaskInfo mask;
        glm::mat4 model;
    };

    struct PanelKeyComparator {
        bool operator()(const PanelKey& a, const PanelKey& b) const;
    };

    struct ScissorBatch {
        std::vector<PanelVertex> vertices;
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
    std::map<PanelKey, ScissorBatch, PanelKeyComparator> panelBatches_;
    glm::mat4 currentPanelModelMatrix_ = glm::mat4(1.0f);
};

} // namespace Urbaxio

