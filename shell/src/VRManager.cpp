// Ensure experimental GLM features are enabled before any GLM headers are pulled in
#define GLM_ENABLE_EXPERIMENTAL
#include "VRManager.h"
#include <SDL2/SDL.h>
#include <SDL2/SDL_syswm.h>
#include <iostream>
#include <vector>
#include <stdexcept>
#include <algorithm>
#include <cstring>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/transform.hpp> // For glm::scale, glm::translate
#include <glm/gtx/vector_angle.hpp> // <-- FIX: Add missing header for orientedAngle
#include <cmath> // For std::sin, std::cos

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
    if (!CheckGraphicsRequirements()) return false;
    if (!CreateSession(window)) return false;
    if (!CreateAppSpace()) return false;
    if (!CreateActions()) return false;
    if (!CreateControllerSpaces()) return false;
    if (!AttachActionSets()) return false;
    if (!CreateSwapchains()) return false;

    initialized = true;
    std::cout << "VRManager: Initialization successful." << std::endl;
    
    // Poll events to get initial state
    std::cout << "VRManager: Polling for initial session state..." << std::endl;
    for (int i = 0; i < 10; ++i) {
        PollEvents();
        if (sessionRunning) break;
        SDL_Delay(10);
    }
    
    if (sessionRunning) {
        std::cout << "VRManager: Session is now running!" << std::endl;
    } else {
        std::cout << "VRManager: Session not yet running (state = " << sessionState << "), will continue polling in main loop" << std::endl;
    }
    
    return true;
}

void VRManager::Shutdown() {
    if (!initialized) return;

    // Clean up OpenGL resources first
    for (auto& sc : swapchains) {
        if (!sc.fbos.empty()) {
            glDeleteFramebuffers((GLsizei)sc.fbos.size(), sc.fbos.data());
        }
        if (!sc.depthBuffers.empty()) {
            glDeleteRenderbuffers((GLsizei)sc.depthBuffers.size(), sc.depthBuffers.data());
        }
        if (sc.swapchain != XR_NULL_HANDLE) {
            xrDestroySwapchain(sc.swapchain);
        }
    }
    swapchains.clear();

    // Clean up actions and spaces
    if (leftGripSpace != XR_NULL_HANDLE) xrDestroySpace(leftGripSpace);
    if (rightGripSpace != XR_NULL_HANDLE) xrDestroySpace(rightGripSpace);
    if (controllerPoseAction != XR_NULL_HANDLE) xrDestroyAction(controllerPoseAction);
    if (aButtonAction != XR_NULL_HANDLE) xrDestroyAction(aButtonAction);
    if (undoRedoActivationAction_ != XR_NULL_HANDLE) xrDestroyAction(undoRedoActivationAction_);
    if (hapticAction_ != XR_NULL_HANDLE) xrDestroyAction(hapticAction_);
    if (triggerValueAction != XR_NULL_HANDLE) xrDestroyAction(triggerValueAction);
    if (squeezeValueAction != XR_NULL_HANDLE) xrDestroyAction(squeezeValueAction);
    if (actionSet != XR_NULL_HANDLE) xrDestroyActionSet(actionSet);

    if (appSpace != XR_NULL_HANDLE) xrDestroySpace(appSpace);
    if (session != XR_NULL_HANDLE) xrDestroySession(session); // Note: xrEndSession is implicitly called by xrDestroySession if needed
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
    appInfo.apiVersion = XR_MAKE_VERSION(1, 0, 0); // Use explicit version instead of XR_CURRENT_API_VERSION

    // List available extensions
    uint32_t extensionCount = 0;
    xrEnumerateInstanceExtensionProperties(nullptr, 0, &extensionCount, nullptr);
    std::vector<XrExtensionProperties> availableExtensions(extensionCount, {XR_TYPE_EXTENSION_PROPERTIES});
    xrEnumerateInstanceExtensionProperties(nullptr, extensionCount, &extensionCount, availableExtensions.data());
    
    std::cout << "VRManager: Available OpenXR extensions (" << extensionCount << "):" << std::endl;
    bool openglSupported = false;
    for (const auto& ext : availableExtensions) {
        std::cout << "  - " << ext.extensionName << std::endl;
        if (strcmp(ext.extensionName, XR_KHR_OPENGL_ENABLE_EXTENSION_NAME) == 0) {
            openglSupported = true;
        }
    }
    
    if (!openglSupported) {
        std::cerr << "VRManager: ERROR - OpenGL extension (XR_KHR_opengl_enable) is NOT supported by this runtime!" << std::endl;
        std::cerr << "VRManager: This application requires OpenGL. Virtual Desktop may not support OpenGL." << std::endl;
        return false;
    }

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

bool VRManager::CheckGraphicsRequirements() {
    // Get the function pointer for xrGetOpenGLGraphicsRequirementsKHR
    PFN_xrGetOpenGLGraphicsRequirementsKHR pfnGetOpenGLGraphicsRequirementsKHR = nullptr;
    XR_CHECK(xrGetInstanceProcAddr(instance, "xrGetOpenGLGraphicsRequirementsKHR",
        (PFN_xrVoidFunction*)&pfnGetOpenGLGraphicsRequirementsKHR), 
        "Failed to get xrGetOpenGLGraphicsRequirementsKHR function pointer");

    XrGraphicsRequirementsOpenGLKHR graphicsRequirements{XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_KHR};
    XR_CHECK(pfnGetOpenGLGraphicsRequirementsKHR(instance, systemId, &graphicsRequirements),
        "Failed to get OpenGL graphics requirements");

    std::cout << "VRManager: OpenGL requirements - Min version: " 
              << XR_VERSION_MAJOR(graphicsRequirements.minApiVersionSupported) << "."
              << XR_VERSION_MINOR(graphicsRequirements.minApiVersionSupported) 
              << ", Max version: "
              << XR_VERSION_MAJOR(graphicsRequirements.maxApiVersionSupported) << "."
              << XR_VERSION_MINOR(graphicsRequirements.maxApiVersionSupported) << std::endl;

    // Check current OpenGL version
    GLint major, minor;
    glGetIntegerv(GL_MAJOR_VERSION, &major);
    glGetIntegerv(GL_MINOR_VERSION, &minor);
    XrVersion currentGLVersion = XR_MAKE_VERSION(major, minor, 0);
    
    std::cout << "VRManager: Current OpenGL version: " << major << "." << minor << std::endl;
    
    if (currentGLVersion < graphicsRequirements.minApiVersionSupported) {
        std::cerr << "VRManager: ERROR - Current OpenGL version " << major << "." << minor 
                  << " is below minimum required " 
                  << XR_VERSION_MAJOR(graphicsRequirements.minApiVersionSupported) << "."
                  << XR_VERSION_MINOR(graphicsRequirements.minApiVersionSupported) << std::endl;
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
    // Use wglGetCurrentDC() to get the DC associated with the current GL context
    // This ensures HDC/HGLRC are from the same device (critical for hybrid GPU laptops)
    hdc = wglGetCurrentDC();
    hglrc = wglGetCurrentContext();
    
    std::cout << "VRManager: HDC = " << (void*)hdc << ", HGLRC = " << (void*)hglrc << std::endl;
    
    if (!hdc || !hglrc) {
        std::cerr << "VR Error: Failed to get HDC or HGLRC from current GL context." << std::endl;
        return false;
    }

    XrGraphicsBindingOpenGLWin32KHR graphicsBinding{XR_TYPE_GRAPHICS_BINDING_OPENGL_WIN32_KHR};
    graphicsBinding.hDC = hdc;
    graphicsBinding.hGLRC = hglrc;

    std::cout << "VRManager: Creating session with systemId = " << systemId << std::endl;

    XrSessionCreateInfo sessionCreateInfo{XR_TYPE_SESSION_CREATE_INFO};
    sessionCreateInfo.next = &graphicsBinding;
    sessionCreateInfo.systemId = systemId;

    XrResult result = xrCreateSession(instance, &sessionCreateInfo, &session);
    if (XR_FAILED(result)) {
        char errorString[XR_MAX_RESULT_STRING_SIZE];
        xrResultToString(instance, result, errorString);
        std::cerr << "VRManager: xrCreateSession failed with error: " << errorString << std::endl;
        std::cerr << "VRManager: This usually means:" << std::endl;
        std::cerr << "  1. Virtual Desktop is not running in the headset" << std::endl;
        std::cerr << "  2. Application was not launched from Virtual Desktop" << std::endl;
        std::cerr << "  3. Virtual Desktop Streamer on PC is not running" << std::endl;
        return false;
    }
#else
    #error "VRManager: Unsupported platform! Only Win32 is currently implemented."
    return false;
#endif
    std::cout << "VRManager: Session created successfully!" << std::endl;
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
        
        // Create FBOs and depth buffers for all images in this swapchain
        swapchains[i].fbos.resize(imageCount);
        swapchains[i].depthBuffers.resize(imageCount);
        glGenFramebuffers(imageCount, swapchains[i].fbos.data());
        glGenRenderbuffers(imageCount, swapchains[i].depthBuffers.data());
        
        for (uint32_t imgIdx = 0; imgIdx < imageCount; ++imgIdx) {
            // Setup depth buffer
            glBindRenderbuffer(GL_RENDERBUFFER, swapchains[i].depthBuffers[imgIdx]);
            glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, 
                                swapchains[i].width, swapchains[i].height);
            
            // Setup FBO
            glBindFramebuffer(GL_FRAMEBUFFER, swapchains[i].fbos[imgIdx]);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 
                                 swapchains[i].images[imgIdx].image, 0);
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 
                                    swapchains[i].depthBuffers[imgIdx]);
            
            GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
            if (status != GL_FRAMEBUFFER_COMPLETE) {
                std::cerr << "VR: FBO " << imgIdx << " for swapchain " << i << " is not complete! Status: " << status << std::endl;
            }
        }
        
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glBindRenderbuffer(GL_RENDERBUFFER, 0);
    }

    return true;
}

bool VRManager::CreateAppSpace() {
    XrReferenceSpaceCreateInfo spaceCreateInfo{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    spaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;

    // Rotate -90 degrees around the X-axis to convert the application's Z-up world
    // into OpenXR's Y-up reference space.
    const float angle = glm::radians(-90.0f);
    const float half_angle = angle * 0.5f;
    
    spaceCreateInfo.poseInReferenceSpace.orientation.x = std::sin(half_angle);
    spaceCreateInfo.poseInReferenceSpace.orientation.y = 0.0f;
    spaceCreateInfo.poseInReferenceSpace.orientation.z = 0.0f;
    spaceCreateInfo.poseInReferenceSpace.orientation.w = std::cos(half_angle);
    
    // Let the runtime manage the user's height by setting position to identity.
    spaceCreateInfo.poseInReferenceSpace.position = { 0.0f, 0.0f, 0.0f };

    XR_CHECK(xrCreateReferenceSpace(session, &spaceCreateInfo, &appSpace), "Failed to create app space");
    return true;
}

bool VRManager::CreateActions() {
    // 1) Get paths for hands
    XR_CHECK_INIT(xrStringToPath(instance, "/user/hand/left", &leftHandPath), "Failed to get left hand path");
    XR_CHECK_INIT(xrStringToPath(instance, "/user/hand/right", &rightHandPath), "Failed to get right hand path");

    // 2) Create action set
    XrActionSetCreateInfo actionSetCI{XR_TYPE_ACTION_SET_CREATE_INFO};
    strcpy_s(actionSetCI.actionSetName, "gameplay");
    strcpy_s(actionSetCI.localizedActionSetName, "Gameplay");
    actionSetCI.priority = 0;
    XR_CHECK_INIT(xrCreateActionSet(instance, &actionSetCI, &actionSet), "Failed to create action set");

    // 3) Create actions
    XrPath subactionPaths[] = { leftHandPath, rightHandPath };
    { // Scope for trigger action
        XrActionCreateInfo actionCI{XR_TYPE_ACTION_CREATE_INFO};
        actionCI.actionType = XR_ACTION_TYPE_FLOAT_INPUT;
        strcpy_s(actionCI.actionName, "trigger_value");
        strcpy_s(actionCI.localizedActionName, "Trigger Value");
        actionCI.countSubactionPaths = 2;
        actionCI.subactionPaths = subactionPaths;
        XR_CHECK_INIT(xrCreateAction(actionSet, &actionCI, &triggerValueAction), "Failed to create trigger value action");
    }
    { // Scope for squeeze action
        XrActionCreateInfo actionCI{XR_TYPE_ACTION_CREATE_INFO};
        actionCI.actionType = XR_ACTION_TYPE_FLOAT_INPUT;
        strcpy_s(actionCI.actionName, "squeeze_value");
        strcpy_s(actionCI.localizedActionName, "Squeeze Value");
        actionCI.countSubactionPaths = 2;
        actionCI.subactionPaths = subactionPaths;
        XR_CHECK_INIT(xrCreateAction(actionSet, &actionCI, &squeezeValueAction), "Failed to create squeeze value action");
    }
    { // Scope for controller pose action
        XrActionCreateInfo actionCI{XR_TYPE_ACTION_CREATE_INFO};
        actionCI.actionType = XR_ACTION_TYPE_POSE_INPUT;
        strcpy_s(actionCI.actionName, "controller_pose");
        strcpy_s(actionCI.localizedActionName, "Controller Pose");
        actionCI.countSubactionPaths = 2;
        actionCI.subactionPaths = subactionPaths;
        XR_CHECK_INIT(xrCreateAction(actionSet, &actionCI, &controllerPoseAction), "Failed to create pose action");
    }
    { // Scope for 'A' button click action
        XrActionCreateInfo actionCI{XR_TYPE_ACTION_CREATE_INFO};
        actionCI.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
        strcpy_s(actionCI.actionName, "a_button_click");
        strcpy_s(actionCI.localizedActionName, "A Button Click");
        actionCI.countSubactionPaths = 1;
        actionCI.subactionPaths = &rightHandPath;
        XR_CHECK_INIT(xrCreateAction(actionSet, &actionCI, &aButtonAction), "Failed to create 'A' button action");
    }
    { // Scope for Left 'X' button (for undo/redo gesture)
        XrActionCreateInfo actionCI{XR_TYPE_ACTION_CREATE_INFO};
        actionCI.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
        strcpy_s(actionCI.actionName, "undo_redo_activation");
        strcpy_s(actionCI.localizedActionName, "Undo/Redo Activation");
        actionCI.countSubactionPaths = 1;
        actionCI.subactionPaths = &leftHandPath;
        XR_CHECK_INIT(xrCreateAction(actionSet, &actionCI, &undoRedoActivationAction_), "Failed to create undo/redo activation action");
    }
    { // --- NEW: Scope for haptic feedback action ---
        XrActionCreateInfo actionCI{XR_TYPE_ACTION_CREATE_INFO};
        actionCI.actionType = XR_ACTION_TYPE_VIBRATION_OUTPUT;
        strcpy_s(actionCI.actionName, "haptic_feedback");
        strcpy_s(actionCI.localizedActionName, "Haptic Feedback");
        actionCI.countSubactionPaths = 2;
        actionCI.subactionPaths = subactionPaths;
        XR_CHECK_INIT(xrCreateAction(actionSet, &actionCI, &hapticAction_), "Failed to create haptic action");
    }

    // 4) Suggest bindings for Oculus Touch profile
    XrPath oculusTouchProfilePath;
    XR_CHECK_INIT(xrStringToPath(instance, "/interaction_profiles/oculus/touch_controller", &oculusTouchProfilePath), "Failed to get oculus touch profile path");

    std::vector<XrActionSuggestedBinding> bindings;
    {
        XrPath path;
        XR_CHECK_INIT(xrStringToPath(instance, "/user/hand/left/input/trigger/value", &path), "Failed to get path");
        bindings.push_back({ triggerValueAction, path });
        XR_CHECK_INIT(xrStringToPath(instance, "/user/hand/right/input/trigger/value", &path), "Failed to get path");
        bindings.push_back({ triggerValueAction, path });

        XR_CHECK_INIT(xrStringToPath(instance, "/user/hand/left/input/squeeze/value", &path), "Failed to get path");
        bindings.push_back({ squeezeValueAction, path });
        XR_CHECK_INIT(xrStringToPath(instance, "/user/hand/right/input/squeeze/value", &path), "Failed to get path");
        bindings.push_back({ squeezeValueAction, path });

        // Grip Pose
        XR_CHECK_INIT(xrStringToPath(instance, "/user/hand/left/input/grip/pose", &path), "Failed to get path");
        bindings.push_back({ controllerPoseAction, path });
        XR_CHECK_INIT(xrStringToPath(instance, "/user/hand/right/input/grip/pose", &path), "Failed to get path");
        bindings.push_back({ controllerPoseAction, path });

        // A Button
        XR_CHECK_INIT(xrStringToPath(instance, "/user/hand/right/input/a/click", &path), "Failed to get path");
        bindings.push_back({ aButtonAction, path });
        // Left X Button
        XR_CHECK_INIT(xrStringToPath(instance, "/user/hand/left/input/x/click", &path), "Failed to get path");
        bindings.push_back({ undoRedoActivationAction_, path });

        // --- NEW: Haptic binding ---
        XR_CHECK_INIT(xrStringToPath(instance, "/user/hand/left/output/haptic", &path), "Failed to get path");
        bindings.push_back({ hapticAction_, path });
    }

    XrInteractionProfileSuggestedBinding suggestedBindings{XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
    suggestedBindings.interactionProfile = oculusTouchProfilePath;
    suggestedBindings.countSuggestedBindings = static_cast<uint32_t>(bindings.size());
    suggestedBindings.suggestedBindings = bindings.data();
    
    XR_CHECK_INIT(xrSuggestInteractionProfileBindings(instance, &suggestedBindings), "Failed to suggest bindings");
    
    return true;
}

void VRManager::TriggerHaptic(XrPath handPath) {
    XrHapticVibration vibration{XR_TYPE_HAPTIC_VIBRATION};
    vibration.amplitude = 0.5f;
    vibration.duration = 50 * 1000000; // 50ms in nanoseconds
    vibration.frequency = XR_FREQUENCY_UNSPECIFIED;
    XrHapticActionInfo hapticActionInfo{XR_TYPE_HAPTIC_ACTION_INFO, nullptr, hapticAction_, handPath};
    xrApplyHapticFeedback(session, &hapticActionInfo, (XrHapticBaseHeader*)&vibration);
}

bool VRManager::CreateControllerSpaces() {
    // Left hand
    XrActionSpaceCreateInfo spaceCI{XR_TYPE_ACTION_SPACE_CREATE_INFO};
    spaceCI.action = controllerPoseAction;
    spaceCI.poseInActionSpace = {{0,0,0,1},{0,0,0}};
    spaceCI.subactionPath = leftHandPath;
    XR_CHECK_INIT(xrCreateActionSpace(session, &spaceCI, &leftGripSpace), "Failed to create left grip space");
    
    // Right hand
    spaceCI.subactionPath = rightHandPath;
    XR_CHECK_INIT(xrCreateActionSpace(session, &spaceCI, &rightGripSpace), "Failed to create right grip space");

    return true;
}

bool VRManager::AttachActionSets() {
    XrSessionActionSetsAttachInfo attachInfo{XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO};
    attachInfo.countActionSets = 1;
    attachInfo.actionSets = &actionSet;
    XR_CHECK_INIT(xrAttachSessionActionSets(session, &attachInfo), "Failed to attach action sets to session");
    return true;
}

void VRManager::PollEvents() {
    XrEventDataBuffer eventData{XR_TYPE_EVENT_DATA_BUFFER};
    while (xrPollEvent(instance, &eventData) == XR_SUCCESS) {
        switch (eventData.type) {
            case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED: {
                auto* stateChangedEvent = reinterpret_cast<XrEventDataSessionStateChanged*>(&eventData);
                sessionState = stateChangedEvent->state;
                switch (sessionState) {
                    case XR_SESSION_STATE_READY: {
                        XrSessionBeginInfo beginInfo{XR_TYPE_SESSION_BEGIN_INFO};
                        beginInfo.primaryViewConfigurationType = viewConfigType;
                        if (XR_SUCCEEDED(xrBeginSession(session, &beginInfo))) {
                             sessionRunning = true;
                             std::cout << "VRManager: VR session started successfully." << std::endl;
                        }
                        break;
                    }
                    case XR_SESSION_STATE_SYNCHRONIZED:
                    case XR_SESSION_STATE_VISIBLE:
                    case XR_SESSION_STATE_FOCUSED:
                        break;
                    case XR_SESSION_STATE_STOPPING: {
                        sessionRunning = false;
                        xrEndSession(session);
                        break;
                    }
                    case XR_SESSION_STATE_EXITING: {
                        sessionRunning = false;
                        break;
                    }
                    default: break;
                }
                break;
            }
            case XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING: {
                 sessionRunning = false;
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
    XrResult result = xrLocateViews(session, &viewLocateInfo, &viewState, viewCount, &viewCount, views.data());
    
    if (XR_FAILED(result)) {
        std::cerr << "VRManager: xrLocateViews failed: " << result << std::endl;
        return;
    }

    renderViews.resize(viewCount);
    for (uint32_t i = 0; i < viewCount; ++i) {
        renderViews[i].pose = views[i].pose;
        renderViews[i].fov = views[i].fov;

        // Apply the INVERSE of the world transform to the camera's view matrix
        glm::mat4 rawViewMatrix = XrPoseToMat4(views[i].pose);
        renderViews[i].viewMatrix = rawViewMatrix * glm::inverse(worldTransform_);

        renderViews[i].projectionMatrix = XrFovToProjMat4(views[i].fov, 0.1f, 1000.0f);
    }
}

bool VRManager::BeginFrame() {
    PollEvents();
    if (!sessionRunning) return false;

    // This must be called once per frame. It snapshots the controller state for this frame.
    XrActiveActionSet activeSet{ actionSet, XR_NULL_PATH };
    XrActionsSyncInfo syncInfo{ XR_TYPE_ACTIONS_SYNC_INFO };
    syncInfo.countActiveActionSets = 1;
    syncInfo.activeActionSets = &activeSet;
    if(XR_FAILED(xrSyncActions(session, &syncInfo))) {
        std::cerr << "VRManager Error: xrSyncActions failed." << std::endl;
    }

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
            projectionViews[i].subImage.imageArrayIndex = 0;
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
    
    XrResult result = xrEndFrame(session, &endInfo);
    if (XR_FAILED(result)) {
        std::cerr << "VRManager: xrEndFrame failed: " << result << std::endl;
    }
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

void VRManager::PollActions() {
    if (!sessionRunning) {
        leftHandVisual_.isValid = false;
        rightHandVisual_.isValid = false;
        return;
    }

    // --- Locate Controller Spaces ---
    auto locateHand = [&](XrSpace space, HandVisual& hand) {
        XrSpaceLocation location{XR_TYPE_SPACE_LOCATION};
        XrResult res = xrLocateSpace(space, appSpace, frameState.predictedDisplayTime, &location);
        if (XR_SUCCEEDED(res) &&
            (location.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) &&
            (location.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT))
        {
            hand.isValid = true;
            hand.pose = location.pose;
        } else {
            hand.isValid = false;
        }
    };
    locateHand(leftGripSpace, leftHandVisual_);
    locateHand(rightGripSpace, rightHandVisual_);

    // --- Get Action States ---
    auto readFloat = [&](XrAction action, XrPath handPath) -> float {
        XrActionStateGetInfo getInfo{XR_TYPE_ACTION_STATE_GET_INFO};
        getInfo.action = action;
        getInfo.subactionPath = handPath;
        XrActionStateFloat state{XR_TYPE_ACTION_STATE_FLOAT};
        xrGetActionStateFloat(session, &getInfo, &state);
        return (state.isActive) ? state.currentState : 0.0f;
    };
    
    float leftTrigger = readFloat(triggerValueAction, leftHandPath);
    rawLeftTriggerValue = leftTrigger; // Store the raw value
    float leftSqueeze = readFloat(squeezeValueAction, leftHandPath);
    float rightTrigger = readFloat(triggerValueAction, rightHandPath);
    float rightSqueeze = readFloat(squeezeValueAction, rightHandPath);

    auto readBool = [&](XrAction action, XrPath handPath) -> bool {
        XrActionStateGetInfo getInfo{XR_TYPE_ACTION_STATE_GET_INFO};
        getInfo.action = action;
        getInfo.subactionPath = handPath;
        XrActionStateBoolean state{XR_TYPE_ACTION_STATE_BOOLEAN};
        xrGetActionStateBoolean(session, &getInfo, &state);
        return (state.isActive && state.currentState);
    };
    aButtonIsPressed = readBool(aButtonAction, rightHandPath);
    bool isUndoRedoHeld = readBool(undoRedoActivationAction_, leftHandPath);

    // --- Double Click Logic for Left A/X Button ---
    leftAButtonDoubleClicked = false;
    const uint32_t DOUBLE_CLICK_TIME_MS = 300;
    if (isUndoRedoHeld && !leftAWasPressed) {
        uint32_t currentTime = SDL_GetTicks();
        if (currentTime - leftALastPressTime < DOUBLE_CLICK_TIME_MS) {
            leftAButtonDoubleClicked = true;
            leftALastPressTime = 0; // Reset timer to prevent triple clicks
        } else {
            leftALastPressTime = currentTime;
        }
    }
    leftAWasPressed = isUndoRedoHeld;

    // --- Combine and Smooth Values for Visuals ---
    float targetLeftPress = std::max(leftTrigger, leftSqueeze);
    float targetRightPress = std::max(rightTrigger, rightSqueeze);

    const float smoothingFactor = 0.25f;
    leftHandVisual_.pressValue += smoothingFactor * (targetLeftPress - leftHandVisual_.pressValue);
    rightHandVisual_.pressValue += smoothingFactor * (targetRightPress - rightHandVisual_.pressValue);
    
    // --- Update Right Hand Trigger Click State ---
    rightHandVisual_.triggerClicked = false; // Reset every frame
    rightHandVisual_.triggerReleased = false;
    bool isTriggerPressed = (rightTrigger > 0.8f);
    if (isTriggerPressed && !rightHandVisual_.triggerWasPressed) {
        rightHandVisual_.triggerClicked = true;
    }
    if (!isTriggerPressed && rightHandVisual_.triggerWasPressed) {
        rightHandVisual_.triggerReleased = true;
    }
    rightHandVisual_.triggerWasPressed = isTriggerPressed;
    
    // --- NEW: Undo/Redo Gesture Logic ---
    // Reset the public action flag at the start of polling
    triggeredUndoRedoAction = UndoRedoAction::None;

    if (isUndoRedoHeld && !isUndoRedoGestureActive_) {
        // Gesture started
        isUndoRedoGestureActive_ = true;
        if (leftHandVisual_.isValid) {
            gestureStartOrientation_ = leftHandVisual_.pose.orientation;
        }
        currentUndoRedoZone_ = UndoRedoZone::None;
    } else if (!isUndoRedoHeld && isUndoRedoGestureActive_) {
        // Gesture ended
        isUndoRedoGestureActive_ = false;
        if (currentUndoRedoZone_ == UndoRedoZone::InUndoZone) {
            triggeredUndoRedoAction = UndoRedoAction::TriggerUndo;
            TriggerHaptic(leftHandPath);
        } else if (currentUndoRedoZone_ == UndoRedoZone::InRedoZone) {
            triggeredUndoRedoAction = UndoRedoAction::TriggerRedo;
            TriggerHaptic(leftHandPath);
        }
        currentUndoRedoZone_ = UndoRedoZone::None;
    }

    if (isUndoRedoGestureActive_ && leftHandVisual_.isValid) {
        glm::quat startQ(gestureStartOrientation_.w, gestureStartOrientation_.x, gestureStartOrientation_.y, gestureStartOrientation_.z);
        glm::quat currentQ(leftHandVisual_.pose.orientation.w, leftHandVisual_.pose.orientation.x, leftHandVisual_.pose.orientation.y, leftHandVisual_.pose.orientation.z);
        
        // Get forward vectors and project them onto the world XY plane (for yaw calculation)
        glm::vec3 startForward = startQ * glm::vec3(0, 0, -1);
        glm::vec3 currentForward = currentQ * glm::vec3(0, 0, -1);
        startForward.z = 0;
        currentForward.z = 0;

        // Only calculate angle if vectors are valid
        if (glm::length2(startForward) > 1e-6 && glm::length2(currentForward) > 1e-6) {
            startForward = glm::normalize(startForward);
            currentForward = glm::normalize(currentForward);
            
            // Calculate signed angle around world Z-up axis
            float angle = glm::orientedAngle(startForward, currentForward, glm::vec3(0, 0, 1));
            
            UndoRedoZone newZone = UndoRedoZone::None;
            const float UNDO_REDO_ANGLE_THRESHOLD = glm::radians(35.0f); // Increased angle threshold

            // Positive angle is left turn -> Redo (INVERTED)
            if (angle > UNDO_REDO_ANGLE_THRESHOLD) {
                newZone = UndoRedoZone::InRedoZone;
            // Negative angle is right turn -> Undo (INVERTED)
            } else if (angle < -UNDO_REDO_ANGLE_THRESHOLD) {
                newZone = UndoRedoZone::InUndoZone;
            }

            // If we entered a new zone, trigger haptic feedback
            if (newZone != currentUndoRedoZone_) {
                if (newZone != UndoRedoZone::None) {
                    TriggerHaptic(leftHandPath);
                }
                currentUndoRedoZone_ = newZone;
            }
        }
    }

    // --- REVISED: Anchor-based Grab Locomotion with Hysteresis ---
    const float GRAB_ON_THRESHOLD = 0.75f;
    const float GRAB_OFF_THRESHOLD = 0.60f;
    const float ZOOM_MIN = 0.10f;
    const float ZOOM_MAX = 10.0f;
    const float ZOOM_SMOOTH = 0.25f;

    auto wantsToGrab = [&](bool isCurrentlyGrabbing, float squeezeValue) {
        float threshold = isCurrentlyGrabbing ? GRAB_OFF_THRESHOLD : GRAB_ON_THRESHOLD;
        return squeezeValue > threshold;
    };

    bool leftWantsToGrab = wantsToGrab(leftGrabState_.isGrabbing, leftSqueeze);
    bool rightWantsToGrab = wantsToGrab(rightGrabState_.isGrabbing, rightSqueeze);

    auto poseToMat4 = [](const XrPosef& pose) {
        glm::quat q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        glm::vec3 p(pose.position.x, pose.position.y, pose.position.z);
        return glm::translate(p) * glm::toMat4(q);
    };

    // --- Two-Handed Zoom Logic ---
    if (!twoHandZoomState_.active && leftWantsToGrab && rightWantsToGrab && leftHandVisual_.isValid && rightHandVisual_.isValid) {
        twoHandZoomState_.active = true;
        twoHandZoomState_.startWorldTransform = worldTransform_;
        glm::vec3 pL(leftHandVisual_.pose.position.x, leftHandVisual_.pose.position.y, leftHandVisual_.pose.position.z);
        glm::vec3 pR(rightHandVisual_.pose.position.x, rightHandVisual_.pose.position.y, rightHandVisual_.pose.position.z);
        twoHandZoomState_.pivotStartTrackingSpace = 0.5f * (pL + pR);
        twoHandZoomState_.startDistance = glm::max(glm::length(pR - pL), 1e-4f);
        twoHandZoomState_.previousScale = 1.0f;

        leftGrabState_.isGrabbing = false;
        rightGrabState_.isGrabbing = false;
    } else if (twoHandZoomState_.active && (!leftWantsToGrab || !rightWantsToGrab)) {
        twoHandZoomState_.active = false;
    }

    if (twoHandZoomState_.active && leftHandVisual_.isValid && rightHandVisual_.isValid) {
        // Update public flags and fade in alpha
        isTwoHandZooming = true;
        zoomTextAlpha = std::min(1.0f, zoomTextAlpha + 0.1f);

        glm::vec3 pL(leftHandVisual_.pose.position.x, leftHandVisual_.pose.position.y, leftHandVisual_.pose.position.z);
        glm::vec3 pR(rightHandVisual_.pose.position.x, rightHandVisual_.pose.position.y, rightHandVisual_.pose.position.z);
        glm::vec3 pMid = 0.5f * (pL + pR);
        zoomMidPoint = pMid;

        float currentDist = glm::max(glm::length(pR - pL), 1e-4f);
        zoomDistance = currentDist;
        float rawScale = glm::clamp(twoHandZoomState_.startDistance / currentDist, ZOOM_MIN, ZOOM_MAX);
        float s = glm::mix(twoHandZoomState_.previousScale, rawScale, ZOOM_SMOOTH);
        twoHandZoomState_.previousScale = s;

        glm::vec4 pivotScene4 = twoHandZoomState_.startWorldTransform * glm::vec4(twoHandZoomState_.pivotStartTrackingSpace, 1.0f);
        glm::vec3 pivotScene = glm::vec3(pivotScene4);

        worldTransform_ = glm::translate(glm::mat4(1.0f), pivotScene) *
                          glm::scale(glm::mat4(1.0f), glm::vec3(s)) *
                          glm::translate(glm::mat4(1.0f), -pivotScene) *
                          twoHandZoomState_.startWorldTransform;
    } else {
        // Update public flags and fade out alpha
        isTwoHandZooming = false;
        zoomTextAlpha = std::max(0.0f, zoomTextAlpha - 0.1f);

        auto updateGrab = [&](bool isGrabbingNow, GrabState& grabState, const HandVisual& hand) {
            if (isGrabbingNow && !grabState.isGrabbing && hand.isValid) {
                grabState.isGrabbing = true;
                glm::mat4 handToWorld = poseToMat4(hand.pose);
                grabState.worldFromHandAnchor = worldTransform_ * handToWorld;
            } else if (!isGrabbingNow && grabState.isGrabbing) {
                grabState.isGrabbing = false;
            } else if (isGrabbingNow && grabState.isGrabbing && hand.isValid) {
                glm::mat4 handToWorld = poseToMat4(hand.pose);
                worldTransform_ = grabState.worldFromHandAnchor * glm::inverse(handToWorld);
            }
        };

        if (leftWantsToGrab) {
            updateGrab(true, leftGrabState_, leftHandVisual_);
            updateGrab(false, rightGrabState_, rightHandVisual_);
        } else if (rightWantsToGrab) {
            updateGrab(true, rightGrabState_, rightHandVisual_);
            updateGrab(false, leftGrabState_, leftHandVisual_);
        } else {
            updateGrab(false, leftGrabState_, leftHandVisual_);
            updateGrab(false, rightGrabState_, rightHandVisual_);
        }
    }
}

} // namespace Urbaxio

