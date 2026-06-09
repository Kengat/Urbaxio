#include "UrbaxioUI.h"

#include <glad/glad.h>
#include <RmlUi/Core.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <vector>

namespace { void FileLog(const std::string& m) { std::ofstream f("rmlui_log.txt", std::ios::app); f << m << "\n"; } }

#include "stb_image.h" // declarations only; implementation lives in main.cpp

namespace Urbaxio::UI {
namespace {

// ---------------------------------------------------------------------------
// Resource resolution (mirrors the shell's load_icon path fallbacks).
// ---------------------------------------------------------------------------
std::string ResolveResource(const std::string& rel) {
    // Return an ABSOLUTE, normalized path: RmlUi resolves rcss/image references
    // relative to the document path, and mishandles leading "../" sequences.
    const char* prefixes[] = {"../../resources/", "../../../resources/", "resources/"};
    for (const char* p : prefixes) {
        std::filesystem::path path = std::string(p) + rel;
        if (std::filesystem::exists(path))
            return std::filesystem::weakly_canonical(std::filesystem::absolute(path)).generic_string();
    }
    return std::filesystem::absolute(std::string("../../../resources/") + rel).generic_string();
}

// ---------------------------------------------------------------------------
// System interface: timing + logging.
// ---------------------------------------------------------------------------
class SystemInterface final : public Rml::SystemInterface {
public:
    double GetElapsedTime() override { return SDL_GetTicks() / 1000.0; }
    bool LogMessage(Rml::Log::Type type, const Rml::String& message) override {
        if (type == Rml::Log::LT_ERROR || type == Rml::Log::LT_ASSERT)
            FileLog("[RmlUi error] " + message);
        return true;
    }
};

// ---------------------------------------------------------------------------
// OpenGL 3.3 render interface (premultiplied alpha to match RmlUi 6 vertices).
// ---------------------------------------------------------------------------
struct CompiledGeometry {
    GLuint vao = 0, vbo = 0, ibo = 0;
    GLsizei num_indices = 0;
};

class RenderInterfaceGL3 final : public Rml::RenderInterface {
public:
    bool Init() {
        const char* vs =
            "#version 330 core\n"
            "layout(location=0) in vec2 inPos;\n"
            "layout(location=1) in vec4 inColor;\n"
            "layout(location=2) in vec2 inUV;\n"
            "uniform vec2 u_translation;\n"
            "uniform mat4 u_projection;\n"
            "out vec4 vColor; out vec2 vUV;\n"
            "void main(){ vColor = inColor; vUV = inUV;\n"
            "    gl_Position = u_projection * vec4(inPos + u_translation, 0.0, 1.0); }\n";
        const char* fs =
            "#version 330 core\n"
            "in vec4 vColor; in vec2 vUV;\n"
            "uniform sampler2D u_tex; uniform int u_useTex;\n"
            "out vec4 frag;\n"
            "void main(){ frag = (u_useTex==1) ? vColor * texture(u_tex, vUV) : vColor; }\n";
        program_ = Link(Compile(GL_VERTEX_SHADER, vs), Compile(GL_FRAGMENT_SHADER, fs));
        if (!program_) return false;
        loc_translation_ = glGetUniformLocation(program_, "u_translation");
        loc_projection_ = glGetUniformLocation(program_, "u_projection");
        loc_useTex_ = glGetUniformLocation(program_, "u_useTex");
        loc_tex_ = glGetUniformLocation(program_, "u_tex");
        return true;
    }

    void Shutdown() {
        if (program_) glDeleteProgram(program_);
        program_ = 0;
    }

    // Called once per frame before context->Render().
    void BeginFrame(int width, int height) {
        viewport_height_ = height;
        glViewport(0, 0, width, height);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_CULL_FACE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA); // premultiplied alpha
        glActiveTexture(GL_TEXTURE0);
        glUseProgram(program_);
        glUniform1i(loc_tex_, 0);

        // Orthographic, top-left origin.
        const float w = (float)width, h = (float)height;
        const float proj[16] = {
            2.0f / w, 0.0f,      0.0f, 0.0f,
            0.0f,     -2.0f / h, 0.0f, 0.0f,
            0.0f,     0.0f,      -1.0f, 0.0f,
            -1.0f,    1.0f,      0.0f, 1.0f,
        };
        glUniformMatrix4fv(loc_projection_, 1, GL_FALSE, proj);
    }

    void EndFrame() {
        glDisable(GL_SCISSOR_TEST);
        glBindVertexArray(0);
        glUseProgram(0);
        // Restore the app's default GL state for the next frame's scene pass.
        glEnable(GL_DEPTH_TEST);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    // --- RmlUi required overrides ---
    Rml::CompiledGeometryHandle CompileGeometry(Rml::Span<const Rml::Vertex> vertices,
                                                Rml::Span<const int> indices) override {
        auto* geo = new CompiledGeometry();
        geo->num_indices = (GLsizei)indices.size();
        glGenVertexArrays(1, &geo->vao);
        glGenBuffers(1, &geo->vbo);
        glGenBuffers(1, &geo->ibo);
        glBindVertexArray(geo->vao);
        glBindBuffer(GL_ARRAY_BUFFER, geo->vbo);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Rml::Vertex), vertices.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, geo->ibo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(int), indices.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Rml::Vertex), (void*)offsetof(Rml::Vertex, position));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(Rml::Vertex), (void*)offsetof(Rml::Vertex, colour));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Rml::Vertex), (void*)offsetof(Rml::Vertex, tex_coord));
        glBindVertexArray(0);
        return (Rml::CompiledGeometryHandle)geo;
    }

    void RenderGeometry(Rml::CompiledGeometryHandle handle, Rml::Vector2f translation,
                        Rml::TextureHandle texture) override {
        auto* geo = (CompiledGeometry*)handle;
        if (!geo) return;
        glUniform2f(loc_translation_, translation.x, translation.y);
        if (texture) {
            glUniform1i(loc_useTex_, 1);
            glBindTexture(GL_TEXTURE_2D, (GLuint)texture);
        } else {
            glUniform1i(loc_useTex_, 0);
            glBindTexture(GL_TEXTURE_2D, 0);
        }
        glBindVertexArray(geo->vao);
        glDrawElements(GL_TRIANGLES, geo->num_indices, GL_UNSIGNED_INT, nullptr);
    }

    void ReleaseGeometry(Rml::CompiledGeometryHandle handle) override {
        auto* geo = (CompiledGeometry*)handle;
        if (!geo) return;
        glDeleteVertexArrays(1, &geo->vao);
        glDeleteBuffers(1, &geo->vbo);
        glDeleteBuffers(1, &geo->ibo);
        delete geo;
    }

    Rml::TextureHandle LoadTexture(Rml::Vector2i& dims, const Rml::String& source) override {
        int w = 0, h = 0, channels = 0;
        // RmlUi uses a top-left texture origin, so disable the app's global vertical
        // flip for this load, then restore the app's default (flip = true).
        stbi_set_flip_vertically_on_load(0);
        unsigned char* data = stbi_load(source.c_str(), &w, &h, &channels, 4);
        stbi_set_flip_vertically_on_load(1);
        if (!data) return 0;
        // Premultiply alpha.
        for (int i = 0; i < w * h; ++i) {
            unsigned char a = data[i * 4 + 3];
            data[i * 4 + 0] = (unsigned char)((data[i * 4 + 0] * a) / 255);
            data[i * 4 + 1] = (unsigned char)((data[i * 4 + 1] * a) / 255);
            data[i * 4 + 2] = (unsigned char)((data[i * 4 + 2] * a) / 255);
        }
        dims.x = w; dims.y = h;
        GLuint tex = UploadTexture(data, w, h);
        stbi_image_free(data);
        return (Rml::TextureHandle)tex;
    }

    Rml::TextureHandle GenerateTexture(Rml::Span<const Rml::byte> source, Rml::Vector2i dims) override {
        GLuint tex = UploadTexture(source.data(), dims.x, dims.y);
        return (Rml::TextureHandle)tex;
    }

    void ReleaseTexture(Rml::TextureHandle texture) override {
        GLuint tex = (GLuint)texture;
        if (tex) glDeleteTextures(1, &tex);
    }

    void EnableScissorRegion(bool enable) override {
        if (enable) glEnable(GL_SCISSOR_TEST);
        else glDisable(GL_SCISSOR_TEST);
    }

    void SetScissorRegion(Rml::Rectanglei region) override {
        const int x = region.Left();
        const int y = region.Top();
        const int w = region.Size().x;
        const int h = region.Size().y;
        glScissor(x, viewport_height_ - (y + h), w, h); // flip to GL bottom-left origin
    }

private:
    static GLuint Compile(GLenum type, const char* src) {
        GLuint s = glCreateShader(type);
        glShaderSource(s, 1, &src, nullptr);
        glCompileShader(s);
        GLint ok = 0; glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
        if (!ok) { char log[512]; glGetShaderInfoLog(s, 512, nullptr, log); std::cerr << "[RmlUi] shader: " << log << std::endl; }
        return s;
    }
    static GLuint Link(GLuint vs, GLuint fs) {
        GLuint p = glCreateProgram();
        glAttachShader(p, vs); glAttachShader(p, fs);
        glLinkProgram(p);
        GLint ok = 0; glGetProgramiv(p, GL_LINK_STATUS, &ok);
        if (!ok) { char log[512]; glGetProgramInfoLog(p, 512, nullptr, log); std::cerr << "[RmlUi] link: " << log << std::endl; }
        glDeleteShader(vs); glDeleteShader(fs);
        return ok ? p : 0;
    }
    static GLuint UploadTexture(const unsigned char* data, int w, int h) {
        GLuint tex = 0;
        glGenTextures(1, &tex);
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glBindTexture(GL_TEXTURE_2D, 0);
        return tex;
    }

    GLuint program_ = 0;
    GLint loc_translation_ = -1, loc_projection_ = -1, loc_useTex_ = -1, loc_tex_ = -1;
    int viewport_height_ = 0;
};

// ---------------------------------------------------------------------------
// Module state.
// ---------------------------------------------------------------------------
SystemInterface g_system;
RenderInterfaceGL3 g_render;
Rml::Context* g_context = nullptr;
Rml::ElementDocument* g_document = nullptr;
std::function<void(const std::string&)> g_action_cb;
SDL_Window* g_window = nullptr;

// Custom title-bar drag state for the borderless window.
bool g_dragging = false;
int g_drag_off_x = 0, g_drag_off_y = 0;

// True if the cursor is over a draggable part of the bar (background/brand/spacer),
// i.e. not over an interactive trigger/item/button.
bool IsOverDragRegion() {
    if (!g_context) return false;
    for (Rml::Element* el = g_context->GetHoverElement(); el; el = el->GetParentNode()) {
        if (el->IsClassSet("trigger") || el->IsClassSet("item") || el->IsClassSet("pill") ||
            el->IsClassSet("winbtn") || el->IsClassSet("dropdown") || el->IsClassSet("submenu"))
            return false;
        if (el->IsClassSet("drag")) return true;
    }
    return false;
}

int MapMouseButton(Uint8 sdl_button) {
    switch (sdl_button) {
        case SDL_BUTTON_LEFT: return 0;
        case SDL_BUTTON_RIGHT: return 1;
        case SDL_BUTTON_MIDDLE: return 2;
        default: return 0;
    }
}

} // namespace

bool InitRml(SDL_Window* window, int width, int height) {
    g_window = window;
    Rml::SetSystemInterface(&g_system);
    if (!g_render.Init()) {
        FileLog("render interface init FAILED");
        return false;
    }
    Rml::SetRenderInterface(&g_render);
    if (!Rml::Initialise()) {
        std::cerr << "[RmlUi] Rml::Initialise failed" << std::endl;
        return false;
    }

    // Fonts (Outfit family + monospace for shortcuts).
    const char* fonts[] = {
        "ui/fonts/outfit-400.ttf", "ui/fonts/outfit-500.ttf",
        "ui/fonts/outfit-600.ttf", "ui/fonts/outfit-700.ttf",
        "ui/fonts/jetbrains-mono-500.ttf",
    };
    for (const char* f : fonts) {
        std::string path = ResolveResource(f);
        if (std::filesystem::exists(path)) Rml::LoadFontFace(path);
    }

    g_context = Rml::CreateContext("main", Rml::Vector2i(width, height));
    if (!g_context) {
        std::cerr << "[RmlUi] CreateContext failed" << std::endl;
        return false;
    }

    std::string doc_path = ResolveResource("ui/menu.rml");
    g_document = g_context->LoadDocument(doc_path);
    if (!g_document) {
        FileLog("LoadDocument FAILED: " + doc_path);
        return false;
    }
    g_document->Show();
    return true;
}

void ShutdownRml() {
    if (g_context) { Rml::RemoveContext("main"); g_context = nullptr; g_document = nullptr; }
    Rml::Shutdown();
    g_render.Shutdown();
}

void SetActionCallback(std::function<void(const std::string&)> cb) { g_action_cb = std::move(cb); }

bool ProcessRmlEvent(const SDL_Event& event) {
    if (!g_context) return false;
    switch (event.type) {
        case SDL_MOUSEMOTION:
            g_context->ProcessMouseMove(event.motion.x, event.motion.y, 0);
            if (g_dragging && g_window) {
                int gx, gy;
                SDL_GetGlobalMouseState(&gx, &gy);
                SDL_SetWindowPosition(g_window, gx - g_drag_off_x, gy - g_drag_off_y);
            }
            break;
        case SDL_MOUSEBUTTONDOWN:
            g_context->ProcessMouseButtonDown(MapMouseButton(event.button.button), 0);
            if (event.button.button == SDL_BUTTON_LEFT && g_window && IsOverDragRegion()) {
                if (event.button.clicks == 2) {
                    // Double-click the title bar toggles maximize (handled by the app).
                    if (g_action_cb) g_action_cb("maximize");
                } else {
                    int gx, gy, wx, wy;
                    SDL_GetGlobalMouseState(&gx, &gy);
                    SDL_GetWindowPosition(g_window, &wx, &wy);
                    g_drag_off_x = gx - wx;
                    g_drag_off_y = gy - wy;
                    g_dragging = true;
                }
            }
            break;
        case SDL_MOUSEBUTTONUP:
            g_context->ProcessMouseButtonUp(MapMouseButton(event.button.button), 0);
            if (event.button.button == SDL_BUTTON_LEFT) {
                g_dragging = false;
                // Fire the action of the nearest ancestor carrying an "act" attribute.
                for (Rml::Element* el = g_context->GetHoverElement(); el; el = el->GetParentNode()) {
                    if (el->HasAttribute("act")) {
                        std::string a = el->GetAttribute<Rml::String>("act", "");
                        if (!a.empty() && g_action_cb) g_action_cb(a);
                        break;
                    }
                }
            }
            break;
        case SDL_MOUSEWHEEL:
            g_context->ProcessMouseWheel((float)-event.wheel.y, 0);
            break;
        case SDL_WINDOWEVENT:
            if (event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED ||
                event.window.event == SDL_WINDOWEVENT_RESIZED) {
                g_context->SetDimensions(Rml::Vector2i(event.window.data1, event.window.data2));
            }
            break;
        default:
            break;
    }
    return IsMouseOverUI();
}

bool IsMouseOverUI() {
    if (!g_context) return false;
    Rml::Element* el = g_context->GetHoverElement();
    while (el) {
        if (el->GetId() == "ui") return true;
        el = el->GetParentNode();
    }
    return false;
}

void UpdateRml() {
    if (g_context) g_context->Update();
}

void RenderRml(int width, int height) {
    if (!g_context) return;
    // Size the UI to the actual GL viewport the app just rendered the scene into.
    // This keeps the bar exactly as wide as the rendered framebuffer, regardless of
    // SDL window-size vs drawable-size quirks (high-DPI, resize, maximize).
    GLint vp[4] = {0, 0, 0, 0};
    glGetIntegerv(GL_VIEWPORT, vp);
    int w = vp[2] > 0 ? vp[2] : width;
    int h = vp[3] > 0 ? vp[3] : height;

    // Set size BEFORE Update so the document reflows to the current size this frame.
    g_context->SetDimensions(Rml::Vector2i(w, h));
    g_context->Update();
    g_render.BeginFrame(w, h);
    g_context->Render();
    g_render.EndFrame();
}

} // namespace Urbaxio::UI
