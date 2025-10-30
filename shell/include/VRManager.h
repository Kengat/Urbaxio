#pragma once

// Platform-specific defines for OpenXR
#if defined(_WIN32)
#define NOMINMAX
#define XR_USE_PLATFORM_WIN32
#define XR_USE_GRAPHICS_API_OPENGL
#include <windows.h>
#endif

#include <glad/glad.h>
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <vector>
#include <string>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

// Forward declarations
struct SDL_Window;

namespace Urbaxio {

// Holds per-eye rendering information
struct VRView {
    XrPosef pose;
    XrFovf fov;
    glm::mat4 projectionMatrix;
    glm::mat4 viewMatrix;
};

// Holds an OpenXR swapchain and its associated OpenGL images + FBOs
struct VRSwapchain {
    XrSwapchain swapchain = XR_NULL_HANDLE;
    int32_t width = 0;
    int32_t height = 0;
    std::vector<XrSwapchainImageOpenGLKHR> images;
    std::vector<GLuint> fbos;          // One FBO per swapchain image
    std::vector<GLuint> depthBuffers;  // One depth buffer per FBO
};

// Manages all OpenXR state and the VR frame loop
class VRManager {
public:
    VRManager();
    ~VRManager();

    bool Initialize(SDL_Window* window);
    void Shutdown();

    bool IsInitialized() const;
    bool IsSessionRunning() const;
    
    // Frame loop functions
    bool BeginFrame();
    void EndFrame();
    
    // Get view information for rendering
    const std::vector<VRView>& GetViews() const;
    
    // Get swapchain info for rendering
    uint32_t AcquireSwapchainImage(uint32_t viewIndex);
    void ReleaseSwapchainImage(uint32_t viewIndex);
    const VRSwapchain& GetSwapchain(uint32_t viewIndex) const;

private:
    bool initialized = false;

    // OpenXR core handles
    XrInstance instance = XR_NULL_HANDLE;
    XrSession session = XR_NULL_HANDLE;
    XrSystemId systemId = XR_NULL_SYSTEM_ID;
    XrSpace appSpace = XR_NULL_HANDLE; // World space (STAGE or LOCAL)
    XrSessionState sessionState = XR_SESSION_STATE_UNKNOWN;
    
    // Graphics binding (Windows specific)
#if defined(XR_USE_PLATFORM_WIN32)
    HDC hdc = nullptr;
    HGLRC hglrc = nullptr;
#endif

    // Frame state
    XrFrameState frameState{XR_TYPE_FRAME_STATE};
    bool frameInProgress = false;
    bool sessionRunning = false;
    std::vector<XrCompositionLayerProjectionView> projectionViews;

    // View and Swapchain data
    XrViewConfigurationType viewConfigType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
    std::vector<XrViewConfigurationView> viewConfigViews;
    std::vector<VRSwapchain> swapchains;
    std::vector<XrView> views; // Raw views from xrLocateViews
    std::vector<VRView> renderViews; // Processed views with matrices

    // Private initialization methods
    bool CreateInstance();
    bool GetSystem();
    bool CheckGraphicsRequirements();
    bool CreateSession(SDL_Window* window);
    bool CreateSwapchains();
    bool CreateAppSpace();

    // Private per-frame methods
    void PollEvents();
    void UpdateViews();
};

} // namespace Urbaxio

