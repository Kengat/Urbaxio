#pragma once

#if defined(_WIN32)
#include <windows.h>
#endif

#include <glad/glad.h>
#include <vector>
#include <string>

namespace Urbaxio {

class DesktopCapture {
public:
    DesktopCapture();
    ~DesktopCapture();

    // Captures the active modal popup of the owner window (e.g., file dialog)
    void Update(void* ownerWindowHandle);

    GLuint GetTextureID() const { return textureID_; }
    bool IsCapturing() const { return isCapturing_; }
    
    int GetWidth() const { return width_; }
    int GetHeight() const { return height_; }
    float GetAspectRatio() const { return (height_ > 0) ? (float)width_ / (float)height_ : 1.0f; }

    // --- NEW: Input Injection ---
    void InjectMouseMove(float u, float v);
    void InjectMouseButton(bool down);
    // ----------------------------

private:
    void Cleanup();
    void RecreateTexture(int w, int h);

    void* targetHWND_ = nullptr;
    GLuint textureID_ = 0;
    int width_ = 0;
    int height_ = 0;
    bool isCapturing_ = false;

    // --- NEW: Track on-screen window position ---
    int screenLeft_ = 0;
    int screenTop_ = 0;
    // --------------------------------------------
    
    // GDI resources
    void* memDC_ = nullptr;     // HDC
    void* memBitmap_ = nullptr; // HBITMAP
    void* oldBitmap_ = nullptr; // HGDIOBJ
    
    std::vector<unsigned char> pixelBuffer_;
};

}

