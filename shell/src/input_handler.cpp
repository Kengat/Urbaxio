#define GLM_ENABLE_EXPERIMENTAL
#include "input_handler.h"
#include "camera.h"
#include <engine/scene.h>
#include <engine/engine.h>
#include <engine/scene_object.h>
#include <cad_kernel/MeshBuffers.h>

#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <SDL2/SDL.h>
#include <iostream>
#include <cmath>
#include <limits>
#include <vector>
#include <algorithm>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <cstddef>
#include <charconv>
#include <string>
#include <cstring>

namespace { // Anonymous namespace for utility functions
    const float LINE_RAY_EPSILON = 1e-6f;
    const float SCREEN_EPSILON = 1e-4f;
    const float SCREEN_VECTOR_MIN_LENGTH_SQ = 4.0f;
    const float LINE_PICK_THRESHOLD_RADIUS = 0.25f;

    // RayTriangleIntersect is now part of SnappingSystem
    // bool RayTriangleIntersect(...) { ... }

    void AppendCharToBuffer(char* buf, size_t bufSize, char c) { /* ... */ size_t len = strlen(buf); if (len + 1 < bufSize) { buf[len] = c; buf[len + 1] = '\0'; } }
    void RemoveLastChar(char* buf) { /* ... */ size_t len = strlen(buf); if (len > 0) { buf[len - 1] = '\0'; } }
    glm::vec3 ClosestPointOnLine(const glm::vec3& lineOrigin, const glm::vec3& lineDir, const glm::vec3& point) { float t = glm::dot(point - lineOrigin, lineDir); return lineOrigin + lineDir * t; }
    const glm::vec3 AXIS_X_DIR(1.0f, 0.0f, 0.0f); const glm::vec3 AXIS_Y_DIR(0.0f, 1.0f, 0.0f); const glm::vec3 AXIS_Z_DIR(0.0f, 0.0f, 1.0f);
    bool IsProjectablePointSnap(Urbaxio::SnapType type) { switch(type) { case Urbaxio::SnapType::ENDPOINT: case Urbaxio::SnapType::ORIGIN: case Urbaxio::SnapType::MIDPOINT: case Urbaxio::SnapType::CENTER: case Urbaxio::SnapType::INTERSECTION: return true; default: return false; } }
}

namespace Urbaxio {

    InputHandler::InputHandler() : middleMouseButtonDown(false), shiftDown(false), shiftWasPressed(false), lastMouseX(0), lastMouseY(0), isMouseFocused(true), firstMouse(true), isAxisLocked(false), lockedAxisType(SnapType::NONE), lockedAxisOrigin(0.0f), lockedAxisDir(0.0f), snappingSystem() {}
    glm::vec3 InputHandler::GetCursorPointInWorld(const Camera& camera, int mouseX, int mouseY, int screenWidth, int screenHeight, const glm::vec3& fallbackPlanePoint) { glm::vec3 point; if (SnappingSystem::RaycastToZPlane(mouseX, mouseY, screenWidth, screenHeight, camera, point)) { return point; } else { glm::vec3 rayOrigin, rayDirection; glm::mat4 view = camera.GetViewMatrix(); glm::mat4 projection = camera.GetProjectionMatrix((screenHeight > 0) ? ((float)screenWidth / (float)screenHeight) : 1.0f); Camera::ScreenToWorldRay(mouseX, mouseY, screenWidth, screenHeight, view, projection, rayOrigin, rayDirection); glm::vec3 planeNormal = -camera.Front; glm::vec3 pointOnPlane = fallbackPlanePoint; float denom = glm::dot(rayDirection, planeNormal); if (std::abs(denom) > 1e-6f) { float t = glm::dot(pointOnPlane - rayOrigin, planeNormal) / denom; if (t > 1e-4f && t < 10000.0f) return rayOrigin + rayDirection * t; } return rayOrigin + rayDirection * 10.0f; } }
    bool InputHandler::RayLineSegmentIntersection( const glm::vec3& rayOrigin, const glm::vec3& rayDir, const glm::vec3& p1, const glm::vec3& p2, float pickThresholdRadius, float& outDistanceAlongRay, glm::vec3& outClosestPointOnSegment ) { /* ... same implementation ... */ glm::vec3 segDir = p2 - p1; float segLenSq = glm::length2(segDir); if (segLenSq < LINE_RAY_EPSILON * LINE_RAY_EPSILON) { outDistanceAlongRay = glm::dot(p1 - rayOrigin, rayDir); if (outDistanceAlongRay < 0) return false; glm::vec3 pointOnRay = rayOrigin + rayDir * outDistanceAlongRay; if (glm::distance2(pointOnRay, p1) < pickThresholdRadius * pickThresholdRadius) { outClosestPointOnSegment = p1; return true; } return false; } glm::vec3 segDirNormalized = glm::normalize(segDir); glm::vec3 w0 = rayOrigin - p1; float a = 1.0f; float b = glm::dot(rayDir, segDirNormalized); float c = 1.0f; float d = glm::dot(rayDir, w0); float e = glm::dot(segDirNormalized, w0); float denom = a * c - b * b; float t_ray, t_seg_param; if (std::abs(denom) < LINE_RAY_EPSILON) { glm::vec3 closest_on_ray_to_p1 = rayOrigin + rayDir * glm::dot(p1 - rayOrigin, rayDir); if (glm::distance2(closest_on_ray_to_p1, p1) < pickThresholdRadius * pickThresholdRadius) { outDistanceAlongRay = glm::dot(p1 - rayOrigin, rayDir); outClosestPointOnSegment = p1; return outDistanceAlongRay >=0; } glm::vec3 closest_on_ray_to_p2 = rayOrigin + rayDir * glm::dot(p2 - rayOrigin, rayDir); if (glm::distance2(closest_on_ray_to_p2, p2) < pickThresholdRadius * pickThresholdRadius) { outDistanceAlongRay = glm::dot(p2 - rayOrigin, rayDir); outClosestPointOnSegment = p2; return outDistanceAlongRay >=0; } return false; } t_ray = (b * e - c * d) / denom; t_seg_param = (a * e - b * d) / denom; float segActualLength = glm::sqrt(segLenSq); t_seg_param = glm::clamp(t_seg_param, 0.0f, segActualLength); outClosestPointOnSegment = p1 + segDirNormalized * t_seg_param; float actual_t_ray = glm::dot(outClosestPointOnSegment - rayOrigin, rayDir); if (actual_t_ray < 0) return false; glm::vec3 closestPointOnRayToClampedSegPoint = rayOrigin + actual_t_ray * rayDir; float distSq = glm::distance2(closestPointOnRayToClampedSegPoint, outClosestPointOnSegment); if (distSq < pickThresholdRadius * pickThresholdRadius) { outDistanceAlongRay = actual_t_ray; return true; } return false; }

    void InputHandler::ProcessEvents( Urbaxio::Camera& camera, bool& should_quit, SDL_Window* window, int& display_w, int& display_h, uint64_t& selectedObjId, size_t& selectedTriangleBaseIndex, std::vector<size_t>& selectedLineIndices, bool isDrawingLineMode, bool& isPlacingFirstPoint, bool& isPlacingSecondPoint, glm::vec3& currentLineStartPoint, Urbaxio::Engine::Scene* scene, glm::vec3& currentRubberBandEnd, SnapResult& currentSnap, char* lineLengthInputBuf, float& lineLengthValue ) {
        // ... (Event loop and most logic remains the same) ...
        SDL_Event event; ImGuiIO& io = ImGui::GetIO(); currentRubberBandEnd = glm::vec3(0.0f); currentSnap.snapped = false; currentSnap.type = SnapType::NONE; bool enterPressedThisFrame = false; bool shiftPressedDownThisFrame = false; bool currentShiftDown = (SDL_GetModState() & KMOD_SHIFT); if (currentShiftDown && !shiftWasPressed) { shiftPressedDownThisFrame = true; } shiftWasPressed = currentShiftDown; shiftDown = currentShiftDown;
        while (SDL_PollEvent(&event)) { ImGui_ImplSDL2_ProcessEvent(&event); bool wantCaptureMouse = io.WantCaptureMouse; bool wantCaptureKeyboard = io.WantCaptureKeyboard; switch (event.type) { case SDL_QUIT: should_quit = true; break; case SDL_WINDOWEVENT: if (event.window.event == SDL_WINDOWEVENT_RESIZED) { SDL_GetWindowSize(window, &display_w, &display_h); } else if (event.window.event == SDL_WINDOWEVENT_FOCUS_GAINED) { isMouseFocused = true; } else if (event.window.event == SDL_WINDOWEVENT_FOCUS_LOST) { isMouseFocused = false; middleMouseButtonDown = false; SDL_ShowCursor(SDL_ENABLE); if (isPlacingSecondPoint || isPlacingFirstPoint) { isPlacingFirstPoint = false; isPlacingSecondPoint = false; isAxisLocked = false; lineLengthInputBuf[0] = '\0'; std::cout << "DEBUG: Line drawing cancelled due to focus loss." << std::endl; } } else if (event.window.event == SDL_WINDOWEVENT_ENTER) { isMouseFocused = true; } else if (event.window.event == SDL_WINDOWEVENT_LEAVE) { isMouseFocused = false; } break; case SDL_KEYDOWN: if (event.key.keysym.sym == SDLK_LSHIFT || event.key.keysym.sym == SDLK_RSHIFT) { shiftDown = true; } if (event.key.keysym.sym == SDLK_ESCAPE) { if (isAxisLocked) { isAxisLocked = false; std::cout << "DEBUG: Axis lock cancelled by ESC." << std::endl; } else if (isPlacingSecondPoint) { isPlacingSecondPoint = false; isPlacingFirstPoint = true; lineLengthInputBuf[0] = '\0'; std::cout << "DEBUG: Line drawing second point cancelled by ESC." << std::endl; } else if (isPlacingFirstPoint) { isPlacingFirstPoint = false; lineLengthInputBuf[0] = '\0'; std::cout << "DEBUG: Line drawing first point placement cancelled by ESC." << std::endl; } } else if (isPlacingSecondPoint && !wantCaptureKeyboard) { SDL_Keycode key = event.key.keysym.sym; bool isEnter = (key == SDLK_RETURN || key == SDLK_KP_ENTER); bool isBackspace = (key == SDLK_BACKSPACE); if (isEnter) { enterPressedThisFrame = true; } else if (isBackspace) { RemoveLastChar(lineLengthInputBuf); } else { char c = '\0'; if ((key >= SDLK_0 && key <= SDLK_9)) { c = (char)key; } else if ((key >= SDLK_KP_0 && key <= SDLK_KP_9)) { c = '0' + (key - SDLK_KP_0); } else if (key == SDLK_PERIOD || key == SDLK_KP_PERIOD) { if (strchr(lineLengthInputBuf, '.') == nullptr) { c = '.'; } } if (c != '\0') { AppendCharToBuffer(lineLengthInputBuf, 64, c); } } } break; case SDL_KEYUP: if (event.key.keysym.sym == SDLK_LSHIFT || event.key.keysym.sym == SDLK_RSHIFT) { shiftDown = false; } break;
        case SDL_MOUSEBUTTONDOWN:
            if (!wantCaptureMouse) {
                if (event.button.button == SDL_BUTTON_MIDDLE) { middleMouseButtonDown = true; firstMouse = true; SDL_ShowCursor(SDL_DISABLE); }
                else if (event.button.button == SDL_BUTTON_LEFT) {
                    int mouseX, mouseY; SDL_GetMouseState(&mouseX, &mouseY);
                    if (isDrawingLineMode && scene) { /* ... line drawing clicks ... */ glm::vec3 clickPoint = currentSnap.worldPoint; if (isPlacingFirstPoint) { currentLineStartPoint = clickPoint; isPlacingFirstPoint = false; isPlacingSecondPoint = true; isAxisLocked = false; lineLengthInputBuf[0] = '\0'; std::cout << "DEBUG: Line Start Point Set: (" << clickPoint.x << ", " << clickPoint.y << ", " << clickPoint.z << ") Snap: " << (currentSnap.snapped ? "Yes" : "No") << std::endl; } else if (isPlacingSecondPoint) { glm::vec3 endPoint = clickPoint; if (glm::distance(currentLineStartPoint, endPoint) > 1e-3f) { scene->AddUserLine(currentLineStartPoint, endPoint); std::cout << "DEBUG: Line Added (Click): Start(...), End(" << endPoint.x << ", " << endPoint.y << ", " << endPoint.z << ") SnapType: " << (int)currentSnap.type << std::endl; } else { std::cout << "DEBUG: Points too close, line not added." << std::endl; } isPlacingSecondPoint = false; isPlacingFirstPoint = true; isAxisLocked = false; lineLengthInputBuf[0] = '\0'; } }
                    else if (!isDrawingLineMode && scene) { // Select lines or objects
                        glm::vec3 rayOrigin, rayDir;
                        Camera::ScreenToWorldRay(mouseX, mouseY, display_w, display_h, camera.GetViewMatrix(), camera.GetProjectionMatrix((float)display_w/(float)display_h), rayOrigin, rayDir);
                        struct LineHit { size_t lineIndex; float distanceAlongRay; glm::vec3 intersectionPoint; };
                        std::vector<LineHit> lineHits;
                        const auto& lineSegments = scene->GetLineSegments();
                        for (size_t i = 0; i < lineSegments.size(); ++i) { const auto& segment = lineSegments[i]; float distAlongRay; glm::vec3 closestPtOnSeg; if (RayLineSegmentIntersection(rayOrigin, rayDir, segment.first, segment.second, LINE_PICK_THRESHOLD_RADIUS, distAlongRay, closestPtOnSeg)) { lineHits.push_back({i, distAlongRay, closestPtOnSeg}); } }
                        if (!lineHits.empty()) { /* ... line selection logic ... */ std::sort(lineHits.begin(), lineHits.end(), [](const LineHit& a, const LineHit& b){ return a.distanceAlongRay < b.distanceAlongRay; }); size_t closestLineIndex = lineHits[0].lineIndex; if (!shiftDown) { selectedLineIndices.clear(); selectedLineIndices.push_back(closestLineIndex); selectedObjId = 0; } else { auto it = std::find(selectedLineIndices.begin(), selectedLineIndices.end(), closestLineIndex); if (it != selectedLineIndices.end()) { selectedLineIndices.erase(it); } else { selectedLineIndices.push_back(closestLineIndex); } } std::cout << "DEBUG: Selected line index: " << closestLineIndex << " (Total selected: " << selectedLineIndices.size() << ")" << std::endl; }
                        else { // No lines hit, try object picking
                            if (!shiftDown) selectedLineIndices.clear();
                            uint64_t hitObjectId = 0; size_t hitTriangleBaseIndex = 0; float closestHitDistance = std::numeric_limits<float>::max();
                            std::vector<Urbaxio::Engine::SceneObject*> objects = scene->get_all_objects();
                            for (Urbaxio::Engine::SceneObject* obj_ptr : objects) {
                                const Urbaxio::Engine::SceneObject& obj = *obj_ptr;
                                if (obj.has_mesh()) {
                                    const auto& mesh = obj.get_mesh_buffers();
                                    const auto& vertices = mesh.vertices;
                                    const auto& indices = mesh.indices;
                                    size_t max_vtx_idx = vertices.size() / 3;
                                    if (max_vtx_idx == 0 || vertices.empty() || indices.empty()) continue;
                                    for (size_t i = 0; i + 2 < indices.size(); i += 3) {
                                        unsigned int i0 = indices[i]; unsigned int i1 = indices[i+1]; unsigned int i2 = indices[i+2];
                                        if (i0 >= max_vtx_idx || i1 >= max_vtx_idx || i2 >= max_vtx_idx) continue;
                                        glm::vec3 v0(vertices[i0*3], vertices[i0*3+1], vertices[i0*3+2]);
                                        glm::vec3 v1(vertices[i1*3], vertices[i1*3+1], vertices[i1*3+2]);
                                        glm::vec3 v2(vertices[i2*3], vertices[i2*3+1], vertices[i2*3+2]);
                                        float t = -1.0f;
                                        // <<< USE SnappingSystem's static method >>>
                                        if (SnappingSystem::RayTriangleIntersect(rayOrigin, rayDir, v0, v1, v2, t)) {
                                            if (t < closestHitDistance && t > 0) { // Ensure t is positive
                                                closestHitDistance = t;
                                                hitObjectId = obj.get_id();
                                                hitTriangleBaseIndex = i;
                                            }
                                        }
                                    }
                                }
                            }
                            selectedObjId = hitObjectId; selectedTriangleBaseIndex = hitTriangleBaseIndex;
                            if (selectedObjId != 0) { std::cout << "DEBUG: Hit Object ID: " << selectedObjId << std::endl; }
                            else { if (!shiftDown) { selectedObjId = 0; } std::cout << "DEBUG: Click missed objects and lines." << std::endl; }
                        }
                    }
                } else if (event.button.button == SDL_BUTTON_RIGHT) { /* ... same cancel logic ... */ if (isAxisLocked) { isAxisLocked = false; std::cout << "DEBUG: Axis lock cancelled by Right Click." << std::endl; } else if (isPlacingSecondPoint) { isPlacingSecondPoint = false; isPlacingFirstPoint = true; lineLengthInputBuf[0] = '\0'; std::cout << "DEBUG: Line drawing second point cancelled by Right Click." << std::endl; } else if (isPlacingFirstPoint) { isPlacingFirstPoint = false; lineLengthInputBuf[0] = '\0'; std::cout << "DEBUG: Line drawing first point placement cancelled by Right Click." << std::endl; } }
            } break;
        case SDL_MOUSEBUTTONUP: /* ... same ... */ if (event.button.button == SDL_BUTTON_MIDDLE) { middleMouseButtonDown = false; SDL_ShowCursor(SDL_ENABLE); } break;
        case SDL_MOUSEWHEEL: /* ... same ... */ if (!wantCaptureMouse) { camera.ProcessMouseScroll(static_cast<float>(event.wheel.y)); } break;
        }} // End Event Loop & Switch

        // --- Update Snapping / Axis Locking (AFTER Event Loop) ---
        // ... (same logic as before for axis lock activation, normal snapping, axis locked snapping with point projection) ...
        int currentMouseX, currentMouseY; SDL_GetMouseState(&currentMouseX, &currentMouseY); glm::mat4 view = camera.GetViewMatrix(); glm::mat4 proj = camera.GetProjectionMatrix((display_h > 0) ? ((float)display_w / (float)display_h) : 1.0f); glm::vec3 cursorWorldPoint = GetCursorPointInWorld(camera, currentMouseX, currentMouseY, display_w, display_h, isPlacingSecondPoint ? currentLineStartPoint : glm::vec3(0.0f));
        if (isPlacingSecondPoint) { if (!shiftDown && isAxisLocked) { isAxisLocked = false; std::cout << "DEBUG: Axis lock released." << std::endl; } else if (shiftPressedDownThisFrame && !isAxisLocked) { glm::vec2 startScreenPos, endScreenPos; bool startVisible = SnappingSystem::WorldToScreen(currentLineStartPoint, view, proj, display_w, display_h, startScreenPos); bool endVisible = SnappingSystem::WorldToScreen(cursorWorldPoint, view, proj, display_w, display_h, endScreenPos); if (startVisible && endVisible) { glm::vec2 rubberBandScreenVec = endScreenPos - startScreenPos; float screenLenSq = glm::length2(rubberBandScreenVec); if (screenLenSq > SCREEN_VECTOR_MIN_LENGTH_SQ) { glm::vec2 rubberBandScreenDir = glm::normalize(rubberBandScreenVec); float maxDot = -1.0f; SnapType bestAxisType = SnapType::NONE; glm::vec3 bestAxis3DDir(0.0f); const std::vector<std::pair<SnapType, glm::vec3>> globalAxes = { {SnapType::AXIS_X, AXIS_X_DIR}, {SnapType::AXIS_Y, AXIS_Y_DIR}, {SnapType::AXIS_Z, AXIS_Z_DIR} }; glm::vec2 originScreenPos; if (SnappingSystem::WorldToScreen(glm::vec3(0.0f), view, proj, display_w, display_h, originScreenPos)) { for (const auto& axisPair : globalAxes) { glm::vec2 axisEndScreenPos; if (SnappingSystem::WorldToScreen(axisPair.second, view, proj, display_w, display_h, axisEndScreenPos)) { glm::vec2 axisScreenVec = axisEndScreenPos - originScreenPos; if (glm::length2(axisScreenVec) > SCREEN_EPSILON * SCREEN_EPSILON) { glm::vec2 axisScreenDir = glm::normalize(axisScreenVec); float dot = abs(glm::dot(rubberBandScreenDir, axisScreenDir)); if (dot > maxDot) { maxDot = dot; bestAxisType = axisPair.first; bestAxis3DDir = axisPair.second; } } } } } if (bestAxisType != SnapType::NONE) { isAxisLocked = true; lockedAxisType = bestAxisType; lockedAxisDir = bestAxis3DDir; std::cout << "DEBUG: Axis direction lock activated (Visual): " << (int)lockedAxisType << std::endl; currentSnap.worldPoint = ClosestPointOnLine(currentLineStartPoint, lockedAxisDir, cursorWorldPoint); currentSnap.snapped = true; currentSnap.type = lockedAxisType; currentRubberBandEnd = currentSnap.worldPoint; goto skip_normal_snapping_update_label; } else { std::cout << "DEBUG: Shift pressed but no dominant screen axis found." << std::endl; } } else { std::cout << "DEBUG: Shift pressed but screen line direction is too short." << std::endl; } } else { std::cout << "DEBUG: Shift pressed but start/end points not visible on screen." << std::endl; } } } else { isAxisLocked = false; }
        if (isAxisLocked) { glm::vec3 baseProjectedPoint = ClosestPointOnLine(currentLineStartPoint, lockedAxisDir, cursorWorldPoint); SnapResult cursorSnapResult = snappingSystem.FindSnapPoint(currentMouseX, currentMouseY, display_w, display_h, camera, *scene); if (cursorSnapResult.snapped && IsProjectablePointSnap(cursorSnapResult.type)) { glm::vec3 targetPoint = cursorSnapResult.worldPoint; currentSnap.worldPoint = ClosestPointOnLine(currentLineStartPoint, lockedAxisDir, targetPoint); currentSnap.snapped = true; currentSnap.type = cursorSnapResult.type; } else { currentSnap.worldPoint = baseProjectedPoint; currentSnap.snapped = true; currentSnap.type = lockedAxisType; } currentRubberBandEnd = currentSnap.worldPoint; }
        else if ((isPlacingFirstPoint || isPlacingSecondPoint) && scene) { currentSnap = snappingSystem.FindSnapPoint(currentMouseX, currentMouseY, display_w, display_h, camera, *scene); currentRubberBandEnd = currentSnap.worldPoint; }
        else { currentSnap.snapped = false; currentRubberBandEnd = cursorWorldPoint; }
        skip_normal_snapping_update_label:;

        // --- Handle Line Length Input Confirmation ---
        if (enterPressedThisFrame && isPlacingSecondPoint && scene) { /* ... same length parsing logic ... */ float length = 0.0f; auto [ptr, ec] = std::from_chars(lineLengthInputBuf, lineLengthInputBuf + strlen(lineLengthInputBuf), length); if (ec == std::errc() && ptr == lineLengthInputBuf + strlen(lineLengthInputBuf) && length > 1e-4f) { lineLengthValue = length; glm::vec3 direction; if (isAxisLocked) { float dotProd = glm::dot(currentRubberBandEnd - currentLineStartPoint, lockedAxisDir); direction = (dotProd >= 0.0f) ? lockedAxisDir : -lockedAxisDir; } else { direction = currentRubberBandEnd - currentLineStartPoint; if (glm::length(direction) < 1e-6f) { std::cerr << "Warning: Cannot determine line direction for length input." << std::endl; isPlacingSecondPoint = false; isPlacingFirstPoint = true; isAxisLocked = false; lineLengthInputBuf[0] = '\0'; goto end_event_processing_final_label_2; } direction = glm::normalize(direction); } glm::vec3 finalEndPoint = currentLineStartPoint + direction * lineLengthValue; scene->AddUserLine(currentLineStartPoint, finalEndPoint); std::cout << "DEBUG: Line Added (Length Input): Length=" << lineLengthValue << ", Dir=(" << direction.x << "," << direction.y << "," << direction.z << ")" << std::endl; isPlacingSecondPoint = false; isPlacingFirstPoint = true; isAxisLocked = false; lineLengthInputBuf[0] = '\0'; } else { std::cerr << "Warning: Invalid line length input: '" << lineLengthInputBuf << "'" << std::endl; lineLengthInputBuf[0] = '\0'; } }
        end_event_processing_final_label_2:;

        HandleMouseMotion(camera, window, display_w, display_h);
    }
    void InputHandler::HandleMouseMotion(Urbaxio::Camera& camera, SDL_Window* window, int display_w, int display_h) { /* ... */ int cX, cY; SDL_GetMouseState(&cX, &cY); if (middleMouseButtonDown && isMouseFocused) { if (firstMouse) { lastMouseX = cX; lastMouseY = cY; firstMouse = false; } else { float dX = static_cast<float>(cX - lastMouseX); float dY = static_cast<float>(cY - lastMouseY); if (std::abs(dX) > 1e-3f || std::abs(dY) > 1e-3f) { if (shiftDown) { camera.ProcessPan(dX, dY); } else { camera.ProcessOrbit(dX, dY); } int nX = cX; int nY = cY; bool w = false; const int m = 1; if (display_w > (m + 1) * 2 && display_h > (m + 1) * 2) { if (cX <= m) { nX = display_w - (m + 2); w = true; } else if (cX >= display_w - (m + 1)) { nX = m + 1; w = true; } if (cY <= m) { nY = display_h - (m + 2); w = true; } else if (cY >= display_h - (m + 1)) { nY = m + 1; w = true; } } if (w) { SDL_WarpMouseInWindow(window, nX, nY); lastMouseX = nX; lastMouseY = nY; } else { lastMouseX = cX; lastMouseY = cY; } } else { lastMouseX = cX; lastMouseY = cY; } } } else { firstMouse = true; lastMouseX = cX; lastMouseY = cY; } }

} // namespace Urbaxio