#include "TextRenderer.h"

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <fstream>
#include <iostream>
#include <cstddef>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include "json.hpp"

using json = nlohmann::json;

namespace {
    static GLuint CompileShader(GLenum type, const char* source) {
        GLuint shader = glCreateShader(type);
        glShaderSource(shader, 1, &source, nullptr);
        glCompileShader(shader);
        GLint ok = GL_FALSE;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
        if (!ok) {
            glDeleteShader(shader);
            return 0;
        }
        return shader;
    }
    static GLuint Link(GLuint vs, GLuint fs) {
        GLuint prog = glCreateProgram();
        glAttachShader(prog, vs);
        glAttachShader(prog, fs);
        glLinkProgram(prog);
        glDeleteShader(vs);
        glDeleteShader(fs);
        GLint ok = GL_FALSE;
        glGetProgramiv(prog, GL_LINK_STATUS, &ok);
        if (!ok) {
            glDeleteProgram(prog);
            return 0;
        }
        return prog;
    }
}

namespace Urbaxio {

TextRenderer::TextRenderer() {}
TextRenderer::~TextRenderer() { Shutdown(); }

bool TextRenderer::Initialize(const std::string& fontJsonPath, const std::string& atlasImagePath) {
    if (!CreateShaderProgram()) return false;
    if (!LoadFont(fontJsonPath, atlasImagePath)) return false;

    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, 20000 * sizeof(Vertex), nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, pos));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, uv));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, color));
    glBindVertexArray(0);

    // --- Resources for Panel Text ---
    glGenVertexArrays(1, &panel_vao_);
    glGenBuffers(1, &panel_vbo_);
    glBindVertexArray(panel_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, panel_vbo_);
    glBufferData(GL_ARRAY_BUFFER, 20000 * sizeof(PanelVertex), nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(PanelVertex), (void*)offsetof(PanelVertex, pos));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(PanelVertex), (void*)offsetof(PanelVertex, uv));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(PanelVertex), (void*)offsetof(PanelVertex, color));
    glBindVertexArray(0);

    return true;
}

void TextRenderer::Shutdown() {
    if (shaderProgram_) { glDeleteProgram(shaderProgram_); shaderProgram_ = 0; }
    if (panel_shaderProgram_) { glDeleteProgram(panel_shaderProgram_); panel_shaderProgram_ = 0; }
    if (fontAtlasTexture_) { glDeleteTextures(1, &fontAtlasTexture_); fontAtlasTexture_ = 0; }
    if (vbo_) { glDeleteBuffers(1, &vbo_); vbo_ = 0; }
    if (vao_) { glDeleteVertexArrays(1, &vao_); vao_ = 0; }
    if (panel_vbo_) { glDeleteBuffers(1, &panel_vbo_); panel_vbo_ = 0; }
    if (panel_vao_) { glDeleteVertexArrays(1, &panel_vao_); panel_vao_ = 0; }
}

bool TextRenderer::CreateShaderProgram() {
    const char* vs = R"(
        #version 330 core
        layout(location=0) in vec3 aPos;
        layout(location=1) in vec2 aUv;
        layout(location=2) in vec4 aColor;
        uniform mat4 u_view;
        uniform mat4 u_projection;
        out vec2 vUv;
        out vec4 vColor;
        void main(){
            vUv=aUv; vColor=aColor;
            gl_Position = u_projection * u_view * vec4(aPos,1.0);
        }
    )";
    const char* fs = R"(
        #version 330 core
        in vec2 vUv; in vec4 vColor; out vec4 FragColor;
        uniform sampler2D u_fontAtlas; uniform float u_pxRange;
        float median(float r,float g,float b){ return max(min(r,g), min(max(r,g), b)); }
        void main(){
            vec3 msdf = texture(u_fontAtlas, vUv).rgb;
            float sd = median(msdf.r, msdf.g, msdf.b);
            float screenPxDistance = u_pxRange * (sd - 0.5);
            float opacity = clamp(screenPxDistance + 0.5, 0.0, 1.0);
            if(opacity<0.01) discard;
            FragColor = vec4(vColor.rgb, vColor.a*opacity);
        }
    )";
    GLuint v = CompileShader(GL_VERTEX_SHADER, vs);
    if (!v) return false;
    GLuint f = CompileShader(GL_FRAGMENT_SHADER, fs);
    if (!f) { glDeleteShader(v); return false; }
    shaderProgram_ = Link(v, f);
    if (shaderProgram_ == 0) return false;

    // --- Panel Text Shader ---
    const char* panel_vs = R"(
        #version 330 core
        layout(location=0) in vec3 aPosLocal;
        layout(location=1) in vec2 aUv;
        layout(location=2) in vec4 aColor;
        uniform mat4 u_panelModel;
        uniform mat4 u_view;
        uniform mat4 u_projection;
        out vec2 vUv;
        out vec4 vColor;
        void main(){
            vUv = aUv;
            vColor = aColor;
            gl_Position = u_projection * u_view * u_panelModel * vec4(aPosLocal, 1.0);
        }
    )";

    GLuint panel_v = CompileShader(GL_VERTEX_SHADER, panel_vs);
    if (!panel_v) return false;
    GLuint panel_f = CompileShader(GL_FRAGMENT_SHADER, fs);
    if (!panel_f) { glDeleteShader(panel_v); return false; }
    panel_shaderProgram_ = Link(panel_v, panel_f);
    return panel_shaderProgram_ != 0;
}

bool TextRenderer::LoadFont(const std::string& fontJsonPath, const std::string& atlasImagePath) {
    std::ifstream in(fontJsonPath);
    if (!in.is_open()) return false;
    json j = json::parse(in, nullptr, false);
    if (j.is_discarded()) return false;

    if (j.contains("atlas")) {
        atlasWidth_ = j["atlas"].value("width", 1.0f);
        atlasHeight_ = j["atlas"].value("height", 1.0f);
        pxRange_ = j["atlas"].value("distanceRange", 4.0f);
    }
    if (j.contains("metrics")) {
        fontScale_ = j["metrics"].value("emSize", 32.0f);
        lineHeight_ = j["metrics"].value("lineHeight", 1.0f);
    }
    if (j.contains("glyphs")) {
        for (const auto& gj : j["glyphs"]) {
            Glyph g;
            g.unicode = gj.value("unicode", 0);
            g.advance = gj.value("advance", 0.0f);
            if (gj.contains("planeBounds")) {
                g.planeBounds.x = gj["planeBounds"].value("left", 0.0f);
                g.planeBounds.y = gj["planeBounds"].value("bottom", 0.0f);
                g.planeBounds.z = gj["planeBounds"].value("right", 0.0f);
                g.planeBounds.w = gj["planeBounds"].value("top", 0.0f);
            }
            if (gj.contains("atlasBounds")) {
                g.atlasBounds.x = gj["atlasBounds"].value("left", 0.0f);
                g.atlasBounds.y = gj["atlasBounds"].value("top", 0.0f);
                g.atlasBounds.z = gj["atlasBounds"].value("right", 0.0f);
                g.atlasBounds.w = gj["atlasBounds"].value("bottom", 0.0f);
            }
            glyphs_[g.unicode] = g;
        }
    }

    int w=0,h=0,c=0;
    stbi_set_flip_vertically_on_load(true);
    unsigned char* data = stbi_load(atlasImagePath.c_str(), &w, &h, &c, 3);
    if (!data) return false;
    glGenTextures(1, &fontAtlasTexture_);
    glBindTexture(GL_TEXTURE_2D, fontAtlasTexture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    stbi_image_free(data);
    return true;
}

void TextRenderer::AddText(const std::string& text,
                           const glm::vec3& position,
                           const glm::vec4& color,
                           float scale,
                           const glm::mat4& viewMatrix,
                           bool /*billboard*/) {
    float finalScale = scale / lineHeight_;
    // Normalize billboard basis vectors to avoid scale artifacts from view matrix
    glm::vec3 right = glm::normalize(glm::vec3{ viewMatrix[0][0], viewMatrix[1][0], viewMatrix[2][0] });
    glm::vec3 up    = glm::normalize(glm::vec3{ viewMatrix[0][1], viewMatrix[1][1], viewMatrix[2][1] });

    float totalAdvance = 0.0f;
    for (char ch : text) { auto it = glyphs_.find((int)ch); if (it != glyphs_.end()) totalAdvance += it->second.advance; }
    float xCursor = -totalAdvance * 0.5f;

    for (char ch : text) {
        int uc = (int)ch;
        auto it = glyphs_.find(uc);
        if (it == glyphs_.end()) continue;
        const Glyph& g = it->second;
        if (g.planeBounds == glm::vec4(0.0f)) { xCursor += g.advance; continue; }

        float x0 = xCursor + g.planeBounds.x;
        float y0 = g.planeBounds.y;
        float x1 = xCursor + g.planeBounds.z;
        float y1 = g.planeBounds.w;

        float u0 = g.atlasBounds.x / atlasWidth_;
        float v0 = g.atlasBounds.y / atlasHeight_;
        float u1 = g.atlasBounds.z / atlasWidth_;
        float v1 = g.atlasBounds.w / atlasHeight_;

        glm::vec3 p0 = position + right * (x0 * finalScale) + up * (y0 * finalScale);
        glm::vec3 p1 = position + right * (x1 * finalScale) + up * (y0 * finalScale);
        glm::vec3 p2 = position + right * (x1 * finalScale) + up * (y1 * finalScale);
        glm::vec3 p3 = position + right * (x0 * finalScale) + up * (y1 * finalScale);

        vertices_.push_back({ p0, { u0, v1 }, color });
        vertices_.push_back({ p1, { u1, v1 }, color });
        vertices_.push_back({ p2, { u1, v0 }, color });
        vertices_.push_back({ p0, { u0, v1 }, color });
        vertices_.push_back({ p2, { u1, v0 }, color });
        vertices_.push_back({ p3, { u0, v0 }, color });

        xCursor += g.advance;
    }
}

void TextRenderer::AddTextOnPanel(const std::string& text,
                                const glm::mat4& panelTransform,
                                const glm::vec4& color,
                                float height)
{
    float finalScale = height / lineHeight_;
    glm::vec3 right = glm::vec3(panelTransform[0]);
    glm::vec3 up = glm::vec3(panelTransform[1]);
    float totalAdvance = 0.0f;
    for (char ch : text) {
        auto it = glyphs_.find((int)ch);
        if (it != glyphs_.end()) {
            totalAdvance += it->second.advance;
        }
    }
    float xCursor = -totalAdvance * 0.5f;
    for (char ch : text) {
        int uc = (int)ch;
        auto it = glyphs_.find(uc);
        if (it == glyphs_.end()) continue;
        const Glyph& g = it->second;
        if (g.planeBounds == glm::vec4(0.0f)) {
            xCursor += g.advance;
            continue;
        }
        float x0 = xCursor + g.planeBounds.x;
        float y0 = g.planeBounds.y;
        float x1 = xCursor + g.planeBounds.z;
        float y1 = g.planeBounds.w;
        float u0 = g.atlasBounds.x / atlasWidth_;
        float v0 = g.atlasBounds.y / atlasHeight_;
        float u1 = g.atlasBounds.z / atlasWidth_;
        float v1 = g.atlasBounds.w / atlasHeight_;
        // Positions are relative to the panel's origin (0,0,0 in its local space)
        glm::vec3 p0 = right * (x0 * finalScale) + up * (y0 * finalScale);
        glm::vec3 p1 = right * (x1 * finalScale) + up * (y0 * finalScale);
        glm::vec3 p2 = right * (x1 * finalScale) + up * (y1 * finalScale);
        glm::vec3 p3 = right * (x0 * finalScale) + up * (y1 * finalScale);
        panelVertices_.push_back({ p0, { u0, v1 }, color });
        panelVertices_.push_back({ p1, { u1, v1 }, color });
        panelVertices_.push_back({ p2, { u1, v0 }, color });
        panelVertices_.push_back({ p0, { u0, v1 }, color });
        panelVertices_.push_back({ p2, { u1, v0 }, color });
        panelVertices_.push_back({ p3, { u0, v0 }, color });
        xCursor += g.advance;
    }
}

void TextRenderer::SetPanelModelMatrix(const glm::mat4& modelMatrix) {
    currentPanelModelMatrix_ = modelMatrix;
}

void TextRenderer::Render(const glm::mat4& view, const glm::mat4& projection) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_DEPTH_TEST);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, fontAtlasTexture_);

    // --- Render Billboard Text ---
    if (!vertices_.empty()) {
        glUseProgram(shaderProgram_);
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram_, "u_view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram_, "u_projection"), 1, GL_FALSE, glm::value_ptr(projection));
        glUniform1i(glGetUniformLocation(shaderProgram_, "u_fontAtlas"), 0);
        glUniform1f(glGetUniformLocation(shaderProgram_, "u_pxRange"), pxRange_);

        glBindVertexArray(vao_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferSubData(GL_ARRAY_BUFFER, 0, vertices_.size() * sizeof(Vertex), vertices_.data());
        glDrawArrays(GL_TRIANGLES, 0, (GLsizei)vertices_.size());
    }

    // --- Render Panel Text ---
    if (!panelVertices_.empty()) {
        glUseProgram(panel_shaderProgram_);
        glUniformMatrix4fv(glGetUniformLocation(panel_shaderProgram_, "u_view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(panel_shaderProgram_, "u_projection"), 1, GL_FALSE, glm::value_ptr(projection));
        glUniformMatrix4fv(glGetUniformLocation(panel_shaderProgram_, "u_panelModel"), 1, GL_FALSE, glm::value_ptr(currentPanelModelMatrix_));
        glUniform1i(glGetUniformLocation(panel_shaderProgram_, "u_fontAtlas"), 0);
        glUniform1f(glGetUniformLocation(panel_shaderProgram_, "u_pxRange"), pxRange_);

        glBindVertexArray(panel_vao_);
        glBindBuffer(GL_ARRAY_BUFFER, panel_vbo_);
        glBufferSubData(GL_ARRAY_BUFFER, 0, panelVertices_.size() * sizeof(PanelVertex), panelVertices_.data());
        glDrawArrays(GL_TRIANGLES, 0, (GLsizei)panelVertices_.size());
    }

    glEnable(GL_DEPTH_TEST);
    glBindVertexArray(0);
    vertices_.clear();
    panelVertices_.clear();
}

} // namespace Urbaxio


