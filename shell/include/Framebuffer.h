#pragma once

#include <glad/glad.h>

namespace Urbaxio {

class Framebuffer {
public:
    Framebuffer();
    ~Framebuffer();

    // Delete copy and move constructors/assignments
    Framebuffer(const Framebuffer&) = delete;
    Framebuffer& operator=(const Framebuffer&) = delete;
    Framebuffer(Framebuffer&&) = delete;
    Framebuffer& operator=(Framebuffer&&) = delete;

    bool Create(int width, int height, GLuint colorTexture);
    void Bind();
    static void Unbind();
    void Destroy();

private:
    GLuint fboId = 0;
    GLuint depthRenderbufferId = 0;
    int width = 0;
    int height = 0;
};

} // namespace Urbaxio

