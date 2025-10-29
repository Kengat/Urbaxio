#include "Framebuffer.h"
#include <iostream>

namespace Urbaxio {

Framebuffer::Framebuffer() {}

Framebuffer::~Framebuffer() {
    Destroy();
}

bool Framebuffer::Create(int width, int height, GLuint colorTexture) {
    this->width = width;
    this->height = height;

    glGenFramebuffers(1, &fboId);
    glBindFramebuffer(GL_FRAMEBUFFER, fboId);

    // Attach color texture from the OpenXR swapchain
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colorTexture, 0);

    // Create and attach a depth renderbuffer for 3D rendering
    glGenRenderbuffers(1, &depthRenderbufferId);
    glBindRenderbuffer(GL_RENDERBUFFER, depthRenderbufferId);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthRenderbufferId);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::cerr << "Framebuffer Error: Framebuffer is not complete!" << std::endl;
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        Destroy();
        return false;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return true;
}

void Framebuffer::Bind() {
    glBindFramebuffer(GL_FRAMEBUFFER, fboId);
    glViewport(0, 0, width, height);
}

void Framebuffer::Unbind() {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Framebuffer::Destroy() {
    if (depthRenderbufferId != 0) {
        glDeleteRenderbuffers(1, &depthRenderbufferId);
        depthRenderbufferId = 0;
    }
    if (fboId != 0) {
        glDeleteFramebuffers(1, &fboId);
        fboId = 0;
    }
}

} // namespace Urbaxio

