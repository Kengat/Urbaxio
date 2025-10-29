#include "VRManager.h"
#include <SDL2/SDL.h>
#include <SDL2/SDL_syswm.h>
#include <iostream>
#include <vector>
#include <stdexcept>
#include <algorithm>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>

// Helper macro for checking OpenXR function results during initialization
#define XR_CHECK_INIT(result, message) \
    if (XR_FAILED(result)) { \
        std::cerr << "OpenXR Error on init: " << message << " - " << result << std::endl; \
        return false; \
    }

// Helper macro for checking OpenXR function results after initialization
#define XR_CHECK(result, message) \
    if (XR_FAILED(result)) { \
        char errorString[XR_MAX_RESULT_STRING_SIZE]; \
        xrResultToString(instance, result, errorString); \
        std::cerr << "OpenXR Error: " << message << " - " << errorString << std::endl; \
        Shutdown(); \
        return false; \
    }

namespace { // Anonymous namespace for local helpers
    // Helper to convert XrPosef (position + quaternion) to a 4x4 view matrix
    glm::mat4 XrPoseToMat4(const XrPosef& pose) {
        glm::quat orientation(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        glm::vec3 position(pose.position.x, pose.position.y, pose.position.z);
        glm::mat4 rotationMatrix = glm::toMat4(orientation);
        glm::mat4 translationMatrix = glm::translate(glm::mat4(1.0f), position);
        // The view matrix is the inverse of the camera's transformation
        return glm::inverse(translationMatrix * rotationMatrix);
    }

    // Helper to convert XrFovf (field of view) to a 4x4 projection matrix
    glm::mat4 XrFovToProjMat4(const XrFovf& fov, float nearPlane, float farPlane) {
        float tanLeft = tanf(fov.angleLeft);
        float tanRight = tanf(fov.angleRight);
        float tanDown = tanf(fov.angleDown);
        float tanUp = tanf(fov.angleUp);

        float tanAngleWidth = tanRight - tanLeft;
        float tanAngleHeight = tanUp - tanDown;

        glm::mat4 proj = glm::mat4(0.0f);
        proj[0][0] = 2.0f / tanAngleWidth;
        proj[1][1] = 2.0f / tanAngleHeight;
        proj[2][0] = (tanRight + tanLeft) / tanAngleWidth;
        proj[2][1] = (tanUp + tanDown) / tanAngleHeight;
        proj[2][2] = -(farPlane + nearPlane) / (farPlane - nearPlane);
        proj[2][3] = -1.0f;
        proj[3][2] = -2.0f * farPlane * nearPlane / (farPlane - nearPlane);
        
        return proj;
    }
}

namespace Urbaxio {

VRManager::VRManager() {}

VRManager::~VRManager() {
    Shutdown();
}

bool VRManager::Initialize(SDL_Window* window) {
    if (!CreateInstance()) return false;
    if (!GetSystem()) return false;
    if (!CreateSession(window)) return false;
    if (!CreateAppSpace()) return false;
    if (!CreateSwapchains()) return false;

    initialized = true;
    std::cout << "VRManager: Initialization successful." << std::endl;
    return true;
}

void VRManager::Shutdown() {
    if (!initialized) return;

    for (auto& sc : swapchains) {
        if (sc.swapchain != XR_NULL_HANDLE) xrDestroySwapchain(sc.swapchain);
    }
    swapchains.clear();

    if (appSpace != XR_NULL_HANDLE) xrDestroySpace(appSpace);
    if (session != XR_NULL_HANDLE) xrDestroySession(session);
    if (instance != XR_NULL_HANDLE) xrDestroyInstance(instance);
    
    // Reset all members to default state
    *this = VRManager();
    
    std::cout << "VRManager: Shutdown." << std::endl;
}

bool VRManager::IsInitialized() const { return initialized; }
bool VRManager::IsSessionRunning() const { return sessionRunning; }

bool VRManager::CreateInstance() {
    XrApplicationInfo appInfo{};
    strcpy_s(appInfo.applicationName, "Urbaxio");
    appInfo.applicationVersion = 1;
    strcpy_s(appInfo.engineName, "Urbaxio Engine");
    appInfo.engineVersion = 1;
    appInfo.apiVersion = XR_CURRENT_API_VERSION;

    std::vector<const char*> extensions = { XR_KHR_OPENGL_ENABLE_EXTENSION_NAME };

    XrInstanceCreateInfo createInfo{XR_TYPE_INSTANCE_CREATE_INFO};
    createInfo.applicationInfo = appInfo;
    createInfo.enabledExtensionCount = (uint32_t)extensions.size();
    createInfo.enabledExtensionNames = extensions.data();

    XR_CHECK_INIT(xrCreateInstance(&createInfo, &instance), "Failed to create OpenXR instance");
    return true;
}

bool VRManager::GetSystem() {
    XrSystemGetInfo systemGetInfo{XR_TYPE_SYSTEM_GET_INFO};
    systemGetInfo.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    XR_CHECK(xrGetSystem(instance, &systemGetInfo, &systemId), "Failed to get OpenXR system");

    uint32_t viewConfigCount;
    XR_CHECK(xrEnumerateViewConfigurations(instance, systemId, 0, &viewConfigCount, nullptr), "Failed to get view config count");
    std::vector<XrViewConfigurationType> viewConfigs(viewConfigCount);
    XR_CHECK(xrEnumerateViewConfigurations(instance, systemId, viewConfigCount, &viewConfigCount, viewConfigs.data()), "Failed to enumerate view configs");
    
    auto stereoIt = std::find(viewConfigs.begin(), viewConfigs.end(), XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO);
    if (stereoIt == viewConfigs.end()) {
        std::cerr << "VR Error: Stereo view configuration not supported by the system." << std::endl;
        return false;
    }
    
    return true;
}

bool VRManager::CreateSession(SDL_Window* window) {
    SDL_SysWMinfo wmInfo;
    SDL_VERSION(&wmInfo.version);
    if (!SDL_GetWindowWMInfo(window, &wmInfo)) {
        std::cerr << "VR Error: Could not get SDL window WM info." << std::endl;
        return false;
    }

#if defined(XR_USE_PLATFORM_WIN32)
    hdc = GetDC(wmInfo.info.win.window);
    hglrc = wglGetCurrentContext();
    if (!hdc || !hglrc) {
        std::cerr << "VR Error: Failed to get HDC or HGLRC from main window." << std::endl;
        return false;
    }

    XrGraphicsBindingOpenGLWin32KHR graphicsBinding{XR_TYPE_GRAPHICS_BINDING_OPENGL_WIN32_KHR};
    graphicsBinding.hDC = hdc;
    graphicsBinding.hGLRC = hglrc;

    XrSessionCreateInfo sessionCreateInfo{XR_TYPE_SESSION_CREATE_INFO};
    sessionCreateInfo.next = &graphicsBinding;
    sessionCreateInfo.systemId = systemId;

    XR_CHECK(xrCreateSession(instance, &sessionCreateInfo, &session), "Failed to create OpenXR session");
#else
    #error "VRManager: Unsupported platform! Only Win32 is currently implemented."
    return false;
#endif
    return true;
}

bool VRManager::CreateSwapchains() {
    uint32_t viewCount;
    XR_CHECK(xrEnumerateViewConfigurationViews(instance, systemId, viewConfigType, 0, &viewCount, nullptr), "Failed to get view count");
    viewConfigViews.resize(viewCount, {XR_TYPE_VIEW_CONFIGURATION_VIEW});
    XR_CHECK(xrEnumerateViewConfigurationViews(instance, systemId, viewConfigType, viewCount, &viewCount, viewConfigViews.data()), "Failed to enumerate view config views");

    uint32_t formatCount;
    XR_CHECK(xrEnumerateSwapchainFormats(session, 0, &formatCount, nullptr), "Failed to get swapchain format count");
    std::vector<int64_t> formats(formatCount);
    XR_CHECK(xrEnumerateSwapchainFormats(session, formatCount, &formatCount, formats.data()), "Failed to enumerate swapchain formats");
    
    int64_t chosenFormat = formats[0];
    for (int64_t format : formats) {
        if (format == GL_SRGB8_ALPHA8) {
            chosenFormat = format;
            break;
        }
    }

    swapchains.resize(viewCount);
    for (uint32_t i = 0; i < viewCount; ++i) {
        XrSwapchainCreateInfo swapchainCreateInfo{XR_TYPE_SWAPCHAIN_CREATE_INFO};
        swapchainCreateInfo.usageFlags = XR_SWAPCHAIN_USAGE_SAMPLED_BIT | XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT;
        swapchainCreateInfo.format = chosenFormat;
        swapchainCreateInfo.width = viewConfigViews[i].recommendedImageRectWidth;
        swapchainCreateInfo.height = viewConfigViews[i].recommendedImageRectHeight;
        swapchainCreateInfo.sampleCount = 1; // Multisamping handled by FBOs if needed, not swapchain
        swapchainCreateInfo.faceCount = 1;
        swapchainCreateInfo.arraySize = 1;
        swapchainCreateInfo.mipCount = 1;

        XR_CHECK(xrCreateSwapchain(session, &swapchainCreateInfo, &swapchains[i].swapchain), "Failed to create swapchain");

        swapchains[i].width = swapchainCreateInfo.width;
        swapchains[i].height = swapchainCreateInfo.height;

        uint32_t imageCount;
        XR_CHECK(xrEnumerateSwapchainImages(swapchains[i].swapchain, 0, &imageCount, nullptr), "Failed to get swapchain image count");
        swapchains[i].images.resize(imageCount, {XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_KHR});
        XR_CHECK(xrEnumerateSwapchainImages(swapchains[i].swapchain, imageCount, &imageCount, (XrSwapchainImageBaseHeader*)swapchains[i].images.data()), "Failed to enumerate swapchain images");
    }

    return true;
}

bool VRManager::CreateAppSpace() {
    XrReferenceSpaceCreateInfo spaceCreateInfo{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    spaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
    spaceCreateInfo.poseInReferenceSpace = {{0,0,0,1},{0,1.6f,0}}; // Start at average eye height
    XR_CHECK(xrCreateReferenceSpace(session, &spaceCreateInfo, &appSpace), "Failed to create app space");
    return true;
}

void VRManager::PollEvents() {
    XrEventDataBuffer eventData{XR_TYPE_EVENT_DATA_BUFFER};
    while (xrPollEvent(instance, &eventData) == XR_SUCCESS) {
        switch (eventData.type) {
            case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED: {
                auto* stateChangedEvent = reinterpret_cast<XrEventDataSessionStateChanged*>(&eventData);
                sessionState = stateChangedEvent->state;
                std::cout << "VRManager: Session state changed to " << sessionState << std::endl;
                switch (sessionState) {
                    case XR_SESSION_STATE_READY: {
                        XrSessionBeginInfo beginInfo{XR_TYPE_SESSION_BEGIN_INFO};
                        beginInfo.primaryViewConfigurationType = viewConfigType;
                        if (XR_SUCCEEDED(xrBeginSession(session, &beginInfo))) {
                             sessionRunning = true;
                        }
                        break;
                    }
                    case XR_SESSION_STATE_STOPPING: {
                        sessionRunning = false;
                        xrEndSession(session);
                        break;
                    }
                    case XR_SESSION_STATE_EXITING: {
                        sessionRunning = false;
                        break; // Application should exit
                    }
                    default: break;
                }
                break;
            }
            case XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING: {
                 std::cout << "VRManager: Instance loss pending. Exiting." << std::endl;
                 sessionRunning = false; // Application should exit
                 break;
            }
            default: break;
        }
        eventData = {XR_TYPE_EVENT_DATA_BUFFER};
    }
}

void VRManager::UpdateViews() {
    XrViewLocateInfo viewLocateInfo{XR_TYPE_VIEW_LOCATE_INFO};
    viewLocateInfo.viewConfigurationType = viewConfigType;
    viewLocateInfo.displayTime = frameState.predictedDisplayTime;
    viewLocateInfo.space = appSpace;

    uint32_t viewCount = (uint32_t)viewConfigViews.size();
    views.resize(viewCount, {XR_TYPE_VIEW});
    XrViewState viewState{XR_TYPE_VIEW_STATE};
    xrLocateViews(session, &viewLocateInfo, &viewState, viewCount, &viewCount, views.data());

    renderViews.resize(viewCount);
    for (uint32_t i = 0; i < viewCount; ++i) {
        renderViews[i].pose = views[i].pose;
        renderViews[i].fov = views[i].fov;
        renderViews[i].viewMatrix = XrPoseToMat4(views[i].pose);
        renderViews[i].projectionMatrix = XrFovToProjMat4(views[i].fov, 0.1f, 1000.0f);
    }
}

bool VRManager::BeginFrame() {
    PollEvents();
    if (!sessionRunning) return false;

    frameState = {XR_TYPE_FRAME_STATE};
    if (XR_FAILED(xrWaitFrame(session, nullptr, &frameState))) return false;
    if (XR_FAILED(xrBeginFrame(session, nullptr))) return false;
    
    frameInProgress = frameState.shouldRender;

    if (frameInProgress) {
        UpdateViews();
        projectionViews.resize(renderViews.size(), {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW});
        for (size_t i = 0; i < renderViews.size(); ++i) {
            projectionViews[i].pose = renderViews[i].pose;
            projectionViews[i].fov = renderViews[i].fov;
            projectionViews[i].subImage.swapchain = swapchains[i].swapchain;
            projectionViews[i].subImage.imageRect.offset = {0, 0};
            projectionViews[i].subImage.imageRect.extent = {swapchains[i].width, swapchains[i].height};
        }
    }
    
    return frameInProgress;
}

void VRManager::EndFrame() {
    XrFrameEndInfo endInfo{XR_TYPE_FRAME_END_INFO};
    endInfo.displayTime = frameState.predictedDisplayTime;
    endInfo.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;

    if (frameInProgress) {
        XrCompositionLayerProjection layer{XR_TYPE_COMPOSITION_LAYER_PROJECTION};
        layer.space = appSpace;
        layer.viewCount = (uint32_t)projectionViews.size();
        layer.views = projectionViews.data();
        const XrCompositionLayerBaseHeader* layers[] = { (const XrCompositionLayerBaseHeader*)&layer };
        endInfo.layerCount = 1;
        endInfo.layers = layers;
    } else {
        endInfo.layerCount = 0;
        endInfo.layers = nullptr;
    }
    
    xrEndFrame(session, &endInfo);
    frameInProgress = false;
}

const std::vector<VRView>& VRManager::GetViews() const { return renderViews; }
const VRSwapchain& VRManager::GetSwapchain(uint32_t viewIndex) const { return swapchains[viewIndex]; }

uint32_t VRManager::AcquireSwapchainImage(uint32_t viewIndex) {
    uint32_t imageIndex;
    XrSwapchainImageAcquireInfo acquireInfo{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
    xrAcquireSwapchainImage(swapchains[viewIndex].swapchain, &acquireInfo, &imageIndex);

    XrSwapchainImageWaitInfo waitInfo{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
    waitInfo.timeout = XR_INFINITE_DURATION;
    xrWaitSwapchainImage(swapchains[viewIndex].swapchain, &waitInfo);

    return imageIndex;
}

void VRManager::ReleaseSwapchainImage(uint32_t viewIndex) {
    XrSwapchainImageReleaseInfo releaseInfo{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
    xrReleaseSwapchainImage(swapchains[viewIndex].swapchain, &releaseInfo);
}

} // namespace Urbaxio

