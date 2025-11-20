#include "DesktopCapture.h"
#include <iostream>
#include <vector>
#include <dwmapi.h>

namespace Urbaxio {

DesktopCapture::DesktopCapture() {}

DesktopCapture::~DesktopCapture() {
    Cleanup();
}

void DesktopCapture::Cleanup() {
    if (textureID_ != 0) {
        glDeleteTextures(1, &textureID_);
        textureID_ = 0;
    }
#if defined(_WIN32)
    if (memDC_) {
        SelectObject((HDC)memDC_, (HGDIOBJ)oldBitmap_);
        DeleteDC((HDC)memDC_);
        memDC_ = nullptr;
    }
    if (memBitmap_) {
        DeleteObject((HBITMAP)memBitmap_);
        memBitmap_ = nullptr;
    }
#endif
}

void DesktopCapture::RecreateTexture(int w, int h) {
    if (width_ != w || height_ != h) {
        width_ = w;
        height_ = h;
        
        if (textureID_ == 0) glGenTextures(1, &textureID_);
        
        glBindTexture(GL_TEXTURE_2D, textureID_);
        // Use NEAREST for pixel-perfect desktop look, or LINEAR for smooth
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        // Clamp to edge prevents artifacts at borders
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        
        // Allocate empty
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width_, height_, 0, GL_BGRA, GL_UNSIGNED_BYTE, nullptr);
        
        pixelBuffer_.resize(width_ * height_ * 4);
    }
}

void DesktopCapture::Update(void* ownerWindowHandle) {
#if defined(_WIN32)
    HWND owner = (HWND)ownerWindowHandle;
    HWND foundWnd = GetWindow(owner, GW_ENABLEDPOPUP);
    
    if (foundWnd == owner || foundWnd == NULL || !IsWindowVisible(foundWnd)) {
        isCapturing_ = false;
        return;
    }

    if (targetHWND_ != foundWnd) {
        char title[256];
        GetWindowTextA(foundWnd, title, 256);
        std::cout << "[DesktopCapture] Found Window: '" << title << "' (HWND: " << foundWnd << ")" << std::endl;
    }

    targetHWND_ = foundWnd;
    isCapturing_ = true;

    // 1. Get Exact Visual Bounds on Screen
    // DWM returns the physical coordinates of the window frame *excluding* the shadow.
    RECT visRect;
    HRESULT hr = DwmGetWindowAttribute((HWND)targetHWND_, DWMWA_EXTENDED_FRAME_BOUNDS, &visRect, sizeof(visRect));
    if (FAILED(hr)) {
        GetWindowRect((HWND)targetHWND_, &visRect);
    }
    
    int w = visRect.right - visRect.left;
    int h = visRect.bottom - visRect.top;

    if (w <= 0 || h <= 0) return;

    if (w != width_ || h != height_) {
        std::cout << "[DesktopCapture] Resizing texture to " << w << "x" << h << std::endl;
    }

    RecreateTexture(w, h);

    // 2. Capture from Screen DC
    // This avoids "invisible border" issues because we copy exactly what DWM says is visible.
    HDC screenDC = GetDC(NULL); 
    
    if (!memDC_) {
        memDC_ = CreateCompatibleDC(screenDC);
    }
    
    if (!memBitmap_ || width_ != w || height_ != h) {
        if (memBitmap_) DeleteObject((HBITMAP)memBitmap_);
        memBitmap_ = CreateCompatibleBitmap(screenDC, w, h);
        oldBitmap_ = SelectObject((HDC)memDC_, (HBITMAP)memBitmap_);
    }

    // Copy directly from screen coordinates
    BitBlt((HDC)memDC_, 0, 0, w, h, screenDC, visRect.left, visRect.top, SRCCOPY);
    
    ReleaseDC(NULL, screenDC);

    // 3. Get Bits for OpenGL
    BITMAPINFOHEADER bi = {0};
    bi.biSize = sizeof(BITMAPINFOHEADER);
    bi.biWidth = w;
    bi.biHeight = -h; // Top-down
    bi.biPlanes = 1;
    bi.biBitCount = 32;
    bi.biCompression = BI_RGB;
    
    GetDIBits((HDC)memDC_, (HBITMAP)memBitmap_, 0, h, pixelBuffer_.data(), (BITMAPINFO*)&bi, DIB_RGB_COLORS);
    
    // 4. Force Opaque Alpha
    // Screen capture often leaves alpha as 0 or undefined
    uint8_t* pixels = pixelBuffer_.data();
    size_t totalPixels = w * h;
    for (size_t i = 0; i < totalPixels; ++i) {
        pixels[i * 4 + 3] = 255; 
    }
    
    glBindTexture(GL_TEXTURE_2D, textureID_);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h, GL_BGRA, GL_UNSIGNED_BYTE, pixelBuffer_.data());
#endif
}

}

