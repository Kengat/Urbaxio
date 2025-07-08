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
#include <map>
#include <set>
#include <list>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtx/intersect.hpp> // For glm::intersectRayPlane
#include <cstddef>
#include <charconv>
#include <string>
#include <cstring>

namespace { // Anonymous namespace for utility functions
    const float LINE_RAY_EPSILON = 1e-6f;
    const float SCREEN_EPSILON = 1e-4f;
    const float SCREEN_VECTOR_MIN_LENGTH_SQ = 4.0f;
    const float LINE_PICK_THRESHOLD_RADIUS = 0.25f;
    const float MAX_EXTRUDE_DISTANCE = 1000.0f;

    // Checks if the snap type indicates a point on an existing line that requires a split.
    bool IsLineSplittingSnap(Urbaxio::SnapType type) {
        switch(type) {
            case Urbaxio::SnapType::MIDPOINT:
            case Urbaxio::SnapType::ON_EDGE:
                return true;
            default:
                return false;
        }
    }

    void AppendCharToBuffer(char* buf, size_t bufSize, char c) { /* ... */ size_t len = strlen(buf); if (len + 1 < bufSize) { buf[len] = c; buf[len + 1] = '\0'; } }
    void RemoveLastChar(char* buf) { /* ... */ size_t len = strlen(buf); if (len > 0) { buf[len - 1] = '\0'; } }
    glm::vec3 ClosestPointOnLine(const glm::vec3& lineOrigin, const glm::vec3& lineDir, const glm::vec3& point) { float t = glm::dot(point - lineOrigin, lineDir); return lineOrigin + lineDir * t; }
    const glm::vec3 AXIS_X_DIR(1.0f, 0.0f, 0.0f); const glm::vec3 AXIS_Y_DIR(0.0f, 1.0f, 0.0f); const glm::vec3 AXIS_Z_DIR(0.0f, 0.0f, 1.0f);
    bool IsProjectablePointSnap(Urbaxio::SnapType type) { switch(type) { case Urbaxio::SnapType::ENDPOINT: case Urbaxio::SnapType::ORIGIN: case Urbaxio::SnapType::MIDPOINT: case Urbaxio::SnapType::CENTER: case Urbaxio::SnapType::INTERSECTION: return true; default: return false; } }

    // Check for snaps that are valid for Push/Pull projection
    bool IsValidPushPullSnap(Urbaxio::SnapType type) {
        switch (type) {
            case Urbaxio::SnapType::ENDPOINT:
            case Urbaxio::SnapType::MIDPOINT:
            case Urbaxio::SnapType::ORIGIN:
            case Urbaxio::SnapType::CENTER:
            case Urbaxio::SnapType::AXIS_X:
            case Urbaxio::SnapType::AXIS_Y:
            case Urbaxio::SnapType::AXIS_Z:
                return true;
            default:
                return false;
        }
    }

    std::vector<size_t> FindCoplanarAdjacentTriangles(
        const Urbaxio::Engine::SceneObject& object,
        size_t startTriangleBaseIndex)
    {
        const float NORMAL_DOT_TOLERANCE = 0.999f; // Cosine of angle tolerance
        const float PLANE_DIST_TOLERANCE = 1e-4f;

        const auto& mesh = object.get_mesh_buffers();
        if (!object.has_mesh() || startTriangleBaseIndex + 2 >= mesh.indices.size()) {
            return { startTriangleBaseIndex };
        }

        std::map<std::pair<unsigned int, unsigned int>, std::vector<size_t>> edgeToTriangles;
        for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
            unsigned int v_indices[3] = { mesh.indices[i], mesh.indices[i + 1], mesh.indices[i + 2] };
            for (int j = 0; j < 3; ++j) {
                unsigned int v1_idx = v_indices[j];
                unsigned int v2_idx = v_indices[(j + 1) % 3];
                if (v1_idx > v2_idx) std::swap(v1_idx, v2_idx);
                edgeToTriangles[{v1_idx, v2_idx}].push_back(i);
            }
        }

        std::vector<size_t> resultFaceTriangles;
        std::list<size_t> queue;
        std::set<size_t> visitedTriangles;

        unsigned int i0 = mesh.indices[startTriangleBaseIndex];
        
        glm::vec3 v0(mesh.vertices[i0*3], mesh.vertices[i0*3+1], mesh.vertices[i0*3+2]);
        glm::vec3 referenceNormal(mesh.normals[i0*3], mesh.normals[i0*3+1], mesh.normals[i0*3+2]);
        float referencePlaneD = -glm::dot(referenceNormal, v0);

        queue.push_back(startTriangleBaseIndex);
        visitedTriangles.insert(startTriangleBaseIndex);

        while (!queue.empty()) {
            size_t currentTriangleIndex = queue.front();
            queue.pop_front();
            resultFaceTriangles.push_back(currentTriangleIndex);

            unsigned int current_v_indices[3] = { mesh.indices[currentTriangleIndex], mesh.indices[currentTriangleIndex + 1], mesh.indices[currentTriangleIndex + 2] };

            for (int j = 0; j < 3; ++j) {
                unsigned int v1_idx = current_v_indices[j];
                unsigned int v2_idx = current_v_indices[(j + 1) % 3];
                if (v1_idx > v2_idx) std::swap(v1_idx, v2_idx);

                const auto& potentialNeighbors = edgeToTriangles.at({v1_idx, v2_idx});
                for (size_t neighborIndex : potentialNeighbors) {
                    if (neighborIndex == currentTriangleIndex) continue;

                    if (visitedTriangles.find(neighborIndex) == visitedTriangles.end()) {
                        visitedTriangles.insert(neighborIndex);

                        unsigned int n_i0 = mesh.indices[neighborIndex];
                        
                        glm::vec3 n_v0(mesh.vertices[n_i0*3], mesh.vertices[n_i0*3+1], mesh.vertices[n_i0*3+2]);
                        
                        glm::vec3 neighborNormal(mesh.normals[n_i0*3], mesh.normals[n_i0*3+1], mesh.normals[n_i0*3+2]);
                        
                        if (glm::abs(glm::dot(referenceNormal, neighborNormal)) > NORMAL_DOT_TOLERANCE) {
                            float dist = glm::abs(glm::dot(referenceNormal, n_v0) + referencePlaneD);
                            if (dist < PLANE_DIST_TOLERANCE) {
                                queue.push_back(neighborIndex);
                            }
                        }
                    }
                }
            }
        }
        return resultFaceTriangles;
    }
}

namespace Urbaxio {

    InputHandler::InputHandler() : middleMouseButtonDown(false), shiftDown(false), shiftWasPressed(false), lastMouseX(0), lastMouseY(0), isMouseFocused(true), firstMouse(true), lastClickTimestamp(0), lastClickedObjId(0), lastClickedTriangleIndex(0), pushPull_objId(0), pushPull_startMouseX(0), pushPull_startMouseY(0), isAxisLocked(false), lockedAxisType(SnapType::NONE), lockedAxisOrigin(0.0f), lockedAxisDir(0.0f), snappingSystem() {}
    glm::vec3 InputHandler::GetCursorPointInWorld(const Camera& camera, int mouseX, int mouseY, int screenWidth, int screenHeight, const glm::vec3& fallbackPlanePoint) { glm::vec3 point; if (SnappingSystem::RaycastToZPlane(mouseX, mouseY, screenWidth, screenHeight, camera, point)) { return point; } else { glm::vec3 rayOrigin, rayDirection; glm::mat4 view = camera.GetViewMatrix(); glm::mat4 projection = camera.GetProjectionMatrix((screenHeight > 0) ? ((float)screenWidth / (float)screenHeight) : 1.0f); Camera::ScreenToWorldRay(mouseX, mouseY, screenWidth, screenHeight, view, projection, rayOrigin, rayDirection); glm::vec3 planeNormal = -camera.Front; glm::vec3 pointOnPlane = fallbackPlanePoint; float denom = glm::dot(rayDirection, planeNormal); if (std::abs(denom) > 1e-6f) { float t = glm::dot(pointOnPlane - rayOrigin, planeNormal) / denom; if (t > 1e-4f && t < 10000.0f) return rayOrigin + rayDirection * t; } return rayOrigin + rayDirection * 10.0f; } }
    bool InputHandler::RayLineSegmentIntersection( const glm::vec3& rayOrigin, const glm::vec3& rayDir, const glm::vec3& p1, const glm::vec3& p2, float pickThresholdRadius, float& outDistanceAlongRay, glm::vec3& outClosestPointOnSegment, const std::map<uint64_t, Engine::Line>& lines, uint64_t& outHitLineId ) { /* ... same implementation ... */ glm::vec3 segDir = p2 - p1; float segLenSq = glm::length2(segDir); if (segLenSq < LINE_RAY_EPSILON * LINE_RAY_EPSILON) { outDistanceAlongRay = glm::dot(p1 - rayOrigin, rayDir); if (outDistanceAlongRay < 0) return false; glm::vec3 pointOnRay = rayOrigin + rayDir * outDistanceAlongRay; if (glm::distance2(pointOnRay, p1) < pickThresholdRadius * pickThresholdRadius) { outClosestPointOnSegment = p1; return true; } return false; } glm::vec3 segDirNormalized = glm::normalize(segDir); glm::vec3 w0 = rayOrigin - p1; float a = 1.0f; float b = glm::dot(rayDir, segDirNormalized); float c = 1.0f; float d = glm::dot(rayDir, w0); float e = glm::dot(segDirNormalized, w0); float denom = a * c - b * b; float t_ray, t_seg_param; if (std::abs(denom) < LINE_RAY_EPSILON) { glm::vec3 closest_on_ray_to_p1 = rayOrigin + rayDir * glm::dot(p1 - rayOrigin, rayDir); if (glm::distance2(closest_on_ray_to_p1, p1) < pickThresholdRadius * pickThresholdRadius) { outDistanceAlongRay = glm::dot(p1 - rayOrigin, rayDir); outClosestPointOnSegment = p1; return outDistanceAlongRay >=0; } glm::vec3 closest_on_ray_to_p2 = rayOrigin + rayDir * glm::dot(p2 - rayOrigin, rayDir); if (glm::distance2(closest_on_ray_to_p2, p2) < pickThresholdRadius * pickThresholdRadius) { outDistanceAlongRay = glm::dot(p2 - rayOrigin, rayDir); outClosestPointOnSegment = p2; return outDistanceAlongRay >=0; } return false; } t_ray = (b * e - c * d) / denom; t_seg_param = (a * e - b * d) / denom; float segActualLength = glm::sqrt(segLenSq); t_seg_param = glm::clamp(t_seg_param, 0.0f, segActualLength); outClosestPointOnSegment = p1 + segDirNormalized * t_seg_param; float actual_t_ray = glm::dot(outClosestPointOnSegment - rayOrigin, rayDir); if (actual_t_ray < 0) return false; glm::vec3 closestPointOnRayToClampedSegPoint = rayOrigin + actual_t_ray * rayDir; float distSq = glm::distance2(closestPointOnRayToClampedSegPoint, outClosestPointOnSegment); if (distSq < pickThresholdRadius * pickThresholdRadius) { outDistanceAlongRay = actual_t_ray; return true; } return false; }

    void InputHandler::ProcessEvents(
        Urbaxio::Camera& camera,
        bool& should_quit,
        SDL_Window* window,
        int& display_w,
        int& display_h,
        uint64_t& selectedObjId,
        std::vector<size_t>& selectedTriangleIndices,
        std::set<uint64_t>& selectedLineIDs,
        bool isDrawingLineMode,
        bool isPushPullMode,
        bool& isPushPullActive,
        uint64_t& hoveredObjId,
        std::vector<size_t>& hoveredFaceTriangleIndices,
        float& pushPullCurrentDistance,
        bool& isPlacingFirstPoint,
        bool& isPlacingSecondPoint,
        glm::vec3& currentLineStartPoint,
        Urbaxio::Engine::Scene* scene,
        glm::vec3& currentRubberBandEnd,
        SnapResult& currentSnap,
        char* lineLengthInputBuf,
        float& lineLengthValue
    ) {
        // --- Event Loop ---
        SDL_Event event; ImGuiIO& io = ImGui::GetIO(); bool enterPressedThisFrame = false; bool shiftPressedDownThisFrame = false; bool currentShiftDown = (SDL_GetModState() & KMOD_SHIFT); if (currentShiftDown && !shiftWasPressed) { shiftPressedDownThisFrame = true; } shiftWasPressed = currentShiftDown; shiftDown = currentShiftDown;
        while (SDL_PollEvent(&event)) { ImGui_ImplSDL2_ProcessEvent(&event); bool wantCaptureMouse = io.WantCaptureMouse; bool wantCaptureKeyboard = io.WantCaptureKeyboard; switch (event.type) { case SDL_QUIT: should_quit = true; break; case SDL_WINDOWEVENT: if (event.window.event == SDL_WINDOWEVENT_RESIZED) { SDL_GetWindowSize(window, &display_w, &display_h); } else if (event.window.event == SDL_WINDOWEVENT_FOCUS_GAINED) { isMouseFocused = true; } else if (event.window.event == SDL_WINDOWEVENT_FOCUS_LOST) { isMouseFocused = false; middleMouseButtonDown = false; SDL_ShowCursor(SDL_ENABLE); if (isPlacingSecondPoint || isPlacingFirstPoint) { isPlacingFirstPoint = false; isPlacingSecondPoint = false; isAxisLocked = false; lineLengthInputBuf[0] = '\0'; } if(isPushPullActive) { isPushPullActive = false; } } else if (event.window.event == SDL_WINDOWEVENT_ENTER) { isMouseFocused = true; } else if (event.window.event == SDL_WINDOWEVENT_LEAVE) { isMouseFocused = false; } break; case SDL_KEYDOWN: if (event.key.keysym.sym == SDLK_LSHIFT || event.key.keysym.sym == SDLK_RSHIFT) { shiftDown = true; } if (event.key.keysym.sym == SDLK_ESCAPE) { if (isAxisLocked) { isAxisLocked = false; } else if (isPlacingSecondPoint) { isPlacingSecondPoint = false; isPlacingFirstPoint = true; lineLengthInputBuf[0] = '\0'; } else if (isPlacingFirstPoint) { isPlacingFirstPoint = false; lineLengthInputBuf[0] = '\0'; } else if (isPushPullActive) { isPushPullActive = false; std::cout << "DEBUG: Push/Pull operation cancelled by ESC." << std::endl;} } else if ((isPlacingSecondPoint || isPushPullActive) && !wantCaptureKeyboard) { SDL_Keycode key = event.key.keysym.sym; bool isEnter = (key == SDLK_RETURN || key == SDLK_KP_ENTER); bool isBackspace = (key == SDLK_BACKSPACE); if (isEnter) { enterPressedThisFrame = true; } else if (isBackspace) { RemoveLastChar(lineLengthInputBuf); } else { char c = '\0'; if ((key >= SDLK_0 && key <= SDLK_9)) { c = (char)key; } else if ((key >= SDLK_KP_0 && key <= SDLK_KP_9)) { c = '0' + (key - SDLK_KP_0); } else if (key == SDLK_PERIOD || key == SDLK_KP_PERIOD) { if (strchr(lineLengthInputBuf, '.') == nullptr) { c = '.'; } } if (c != '\0') { AppendCharToBuffer(lineLengthInputBuf, 64, c); } } } break; case SDL_KEYUP: if (event.key.keysym.sym == SDLK_LSHIFT || event.key.keysym.sym == SDLK_RSHIFT) { shiftDown = false; } break;
        case SDL_MOUSEBUTTONDOWN:
            if (!wantCaptureMouse) {
                if (event.button.button == SDL_BUTTON_MIDDLE) { middleMouseButtonDown = true; firstMouse = true; SDL_ShowCursor(SDL_DISABLE); }
                else if (event.button.button == SDL_BUTTON_LEFT) {
                    int mouseX, mouseY; SDL_GetMouseState(&mouseX, &mouseY);
                    if (isDrawingLineMode && scene) { 
                        glm::vec3 clickPoint = currentSnap.worldPoint;
                        if (currentSnap.snapped && IsLineSplittingSnap(currentSnap.type)) {
                            // If we snap to a line's edge/midpoint, split it and use the new vertex.
                            clickPoint = scene->SplitLineAtPoint(currentSnap.snappedEntityId, currentSnap.worldPoint);
                        }
                        
                        if (isPlacingFirstPoint) { 
                            currentLineStartPoint = clickPoint; 
                            isPlacingFirstPoint = false; 
                            isPlacingSecondPoint = true; 
                            isAxisLocked = false; 
                            lineLengthInputBuf[0] = '\0'; 
                        } else if (isPlacingSecondPoint) { 
                            glm::vec3 endPoint = clickPoint; 
                            if (glm::distance2(currentLineStartPoint, endPoint) > 1e-6f) { 
                                scene->AddUserLine(currentLineStartPoint, endPoint); 
                            } 
                            isPlacingSecondPoint = false; 
                            isPlacingFirstPoint = true; 
                            isAxisLocked = false; 
                            lineLengthInputBuf[0] = '\0'; 
                        } 
                    }
                    else if (isPushPullMode && scene) {
                        if (isPushPullActive) {
                            scene->ExtrudeFace(pushPull_objId, pushPull_faceIndices, pushPull_faceNormal, pushPullCurrentDistance);
                            std::cout << "DEBUG: Push/Pull operation finalized with distance: " << pushPullCurrentDistance << std::endl;
                            isPushPullActive = false;
                            pushPullCurrentDistance = 0.0f;

                        } else { // Start a new Push/Pull operation
                            if (hoveredObjId != 0 && !hoveredFaceTriangleIndices.empty()) {
                                isPushPullActive = true;
                                pushPull_objId = hoveredObjId;
                                pushPull_faceIndices = hoveredFaceTriangleIndices;
                                Urbaxio::Engine::SceneObject* obj = scene->get_object_by_id(pushPull_objId);
                                
                                // More robust normal calculation
                                const auto& mesh = obj->get_mesh_buffers();
                                std::set<unsigned int> uniqueVertIndices;
                                for (size_t baseIdx : pushPull_faceIndices) {
                                    uniqueVertIndices.insert(mesh.indices[baseIdx]);
                                    uniqueVertIndices.insert(mesh.indices[baseIdx + 1]);
                                    uniqueVertIndices.insert(mesh.indices[baseIdx + 2]);
                                }
                                glm::vec3 averagedNormal(0.0f);
                                for (unsigned int vIdx : uniqueVertIndices) {
                                    averagedNormal.x += mesh.normals[vIdx * 3];
                                    averagedNormal.y += mesh.normals[vIdx * 3 + 1];
                                    averagedNormal.z += mesh.normals[vIdx * 3 + 2];
                                }
                                if (glm::length2(averagedNormal) > 1e-9f) {
                                    pushPull_faceNormal = glm::normalize(averagedNormal);
                                } else { // Fallback for safety
                                    unsigned int firstTriFirstVertIdx = mesh.indices[pushPull_faceIndices[0]];
                                    pushPull_faceNormal = glm::normalize(glm::vec3(mesh.normals[firstTriFirstVertIdx*3], mesh.normals[firstTriFirstVertIdx*3+1], mesh.normals[firstTriFirstVertIdx*3+2]));
                                }

                                glm::vec3 rayOrigin, rayDir; Camera::ScreenToWorldRay(mouseX, mouseY, display_w, display_h, camera.GetViewMatrix(), camera.GetProjectionMatrix((float)display_w/(float)display_h), rayOrigin, rayDir);
                                float closestHitDist;
                                glm::intersectRayPlane(rayOrigin, rayDir, glm::vec3(mesh.vertices[mesh.indices[pushPull_faceIndices[0]]*3], mesh.vertices[mesh.indices[pushPull_faceIndices[0]]*3+1], mesh.vertices[mesh.indices[pushPull_faceIndices[0]]*3+2]), pushPull_faceNormal, closestHitDist);
                                pushPull_startPoint = rayOrigin + rayDir * closestHitDist;
                                pushPullCurrentDistance = 0.0f;
                                SDL_GetMouseState(&pushPull_startMouseX, &pushPull_startMouseY);

                                selectedObjId = 0; selectedTriangleIndices.clear();
                                lineLengthInputBuf[0] = '\0';
                                std::cout << "DEBUG: Push/Pull operation started on Obj " << pushPull_objId << ", Averaged Normal (" << pushPull_faceNormal.x << ", " << pushPull_faceNormal.y << ", " << pushPull_faceNormal.z << ")" << std::endl;
                            }
                        }
                    }
                    else { // Default selection mode
                        glm::vec3 rayOrigin, rayDir; Camera::ScreenToWorldRay(mouseX, mouseY, display_w, display_h, camera.GetViewMatrix(), camera.GetProjectionMatrix((float)display_w/(float)display_h), rayOrigin, rayDir);
                        uint64_t hitObjectId = 0; size_t hitTriangleBaseIndex = 0; float closestHitDistance = std::numeric_limits<float>::max();
                        struct LineHit { uint64_t lineId; float distanceAlongRay; }; std::vector<LineHit> lineHits; 
                        const auto& all_lines = scene->GetAllLines();
                        for (const auto& [id, line] : all_lines) { float distAlongRay; glm::vec3 closestPtOnSeg; uint64_t hitLineId; if (RayLineSegmentIntersection(rayOrigin, rayDir, line.start, line.end, LINE_PICK_THRESHOLD_RADIUS, distAlongRay, closestPtOnSeg, all_lines, hitLineId)) { lineHits.push_back({id, distAlongRay}); } }
                        
                        if (!lineHits.empty()) { std::sort(lineHits.begin(), lineHits.end(), [](const LineHit& a, const LineHit& b){ return a.distanceAlongRay < b.distanceAlongRay; }); uint64_t closestLineId = lineHits[0].lineId; if (!shiftDown) { selectedLineIDs.clear(); selectedLineIDs.insert(closestLineId); selectedObjId = 0; selectedTriangleIndices.clear(); } else { if (selectedLineIDs.count(closestLineId)) { selectedLineIDs.erase(closestLineId); } else { selectedLineIDs.insert(closestLineId); } }
                        } else {
                            if (!shiftDown) selectedLineIDs.clear();
                            for (Urbaxio::Engine::SceneObject* obj_ptr : scene->get_all_objects()) {
                                if (obj_ptr->has_mesh()) {
                                    const auto& mesh = obj_ptr->get_mesh_buffers();
                                    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
                                        glm::vec3 v0(mesh.vertices[mesh.indices[i]*3], mesh.vertices[mesh.indices[i]*3+1], mesh.vertices[mesh.indices[i]*3+2]); glm::vec3 v1(mesh.vertices[mesh.indices[i+1]*3], mesh.vertices[mesh.indices[i+1]*3+1], mesh.vertices[mesh.indices[i+1]*3+2]); glm::vec3 v2(mesh.vertices[mesh.indices[i+2]*3], mesh.vertices[mesh.indices[i+2]*3+1], mesh.vertices[mesh.indices[i+2]*3+2]); float t; if (SnappingSystem::RayTriangleIntersect(rayOrigin, rayDir, v0, v1, v2, t) && t > 0 && t < closestHitDistance) { closestHitDistance = t; hitObjectId = obj_ptr->get_id(); hitTriangleBaseIndex = i; }
                                        }
                                    }
                                }
                            if (hitObjectId != 0) {
                                uint32_t currentTime = SDL_GetTicks(); const uint32_t DOUBLE_CLICK_TIME = 300;
                                if (currentTime - lastClickTimestamp < DOUBLE_CLICK_TIME && hitObjectId == lastClickedObjId && hitTriangleBaseIndex == lastClickedTriangleIndex) { selectedTriangleIndices.assign(1, hitTriangleBaseIndex); selectedObjId = hitObjectId; lastClickTimestamp = 0;
                                } else { Urbaxio::Engine::SceneObject* hitObject = scene->get_object_by_id(hitObjectId); if (hitObject) { selectedTriangleIndices = FindCoplanarAdjacentTriangles(*hitObject, hitTriangleBaseIndex); selectedObjId = hitObjectId; } lastClickTimestamp = currentTime; lastClickedObjId = hitObjectId; lastClickedTriangleIndex = hitTriangleBaseIndex; }
                            } else { if (!shiftDown) { selectedObjId = 0; selectedTriangleIndices.clear(); } lastClickTimestamp = 0; }
                        }
                    }
                } else if (event.button.button == SDL_BUTTON_RIGHT) { if (isAxisLocked) { isAxisLocked = false; } else if (isPlacingSecondPoint) { isPlacingSecondPoint = false; isPlacingFirstPoint = true; lineLengthInputBuf[0] = '\0'; } else if (isPlacingFirstPoint) { isPlacingFirstPoint = false; lineLengthInputBuf[0] = '\0'; } }
            } break;
        case SDL_MOUSEBUTTONUP: if (event.button.button == SDL_BUTTON_MIDDLE) { middleMouseButtonDown = false; SDL_ShowCursor(SDL_ENABLE); } break;
        case SDL_MOUSEWHEEL: if (!wantCaptureMouse) { camera.ProcessMouseScroll(static_cast<float>(event.wheel.y)); } break;
        }}

        // --- Per-frame updates (after event loop) ---
        int mouseX, mouseY; SDL_GetMouseState(&mouseX, &mouseY);
        if(!isPushPullActive) {
             hoveredObjId = 0; 
             hoveredFaceTriangleIndices.clear();
        }
        currentRubberBandEnd = glm::vec3(0.0f);
        
        if (!io.WantCaptureMouse && scene) {
            if (isPushPullActive) {
                hoveredObjId = pushPull_objId;
                hoveredFaceTriangleIndices = pushPull_faceIndices;
                
                // --- Push/Pull snapping and distance calculation ---
                currentSnap = snappingSystem.FindSnapPoint(mouseX, mouseY, display_w, display_h, camera, *scene);
                if (currentSnap.snapped && IsValidPushPullSnap(currentSnap.type)) {
                    // Project the snapped point onto the extrusion axis to get the distance
                    glm::vec3 projectedPoint = ClosestPointOnLine(pushPull_startPoint, pushPull_faceNormal, currentSnap.worldPoint);
                    glm::vec3 offsetVector = projectedPoint - pushPull_startPoint;
                    pushPullCurrentDistance = glm::dot(offsetVector, pushPull_faceNormal);
                } else {
                    // Fallback to screen-space mouse dragging if no valid snap
                    glm::mat4 view = camera.GetViewMatrix(); glm::mat4 proj = camera.GetProjectionMatrix((float)display_w/(float)display_h);
                    glm::vec2 screenStart, screenEnd;
                    bool p1_visible = SnappingSystem::WorldToScreen(pushPull_startPoint, view, proj, display_w, display_h, screenStart);
                    bool p2_visible = SnappingSystem::WorldToScreen(pushPull_startPoint + pushPull_faceNormal, view, proj, display_w, display_h, screenEnd);
                    
                    if (p1_visible && p2_visible) {
                        glm::vec2 screenAxisDir = screenEnd - screenStart;
                        if (glm::length2(screenAxisDir) > SCREEN_EPSILON * SCREEN_EPSILON) {
                            screenAxisDir = glm::normalize(screenAxisDir);
                            glm::vec2 mouseDelta(mouseX - pushPull_startMouseX, mouseY - pushPull_startMouseY);
                            float pixel_dist = glm::dot(mouseDelta, screenAxisDir);
                            float sensitivity = glm::distance(camera.Position, pushPull_startPoint) * 0.001f;
                            pushPullCurrentDistance = pixel_dist * sensitivity;
                        }
                    }
                }
            } else if (isPushPullMode) {
                // Hover logic only when not actively pushing/pulling
                currentSnap.snapped = false; // No snapping when just hovering for Push/Pull
                glm::vec3 rayOrigin, rayDir; Camera::ScreenToWorldRay(mouseX, mouseY, display_w, display_h, camera.GetViewMatrix(), camera.GetProjectionMatrix((float)display_w/(float)display_h), rayOrigin, rayDir);
                uint64_t currentHoveredObjId = 0; size_t currentHoveredTriangleIdx = 0; float closestHitDist = std::numeric_limits<float>::max();
                for (Urbaxio::Engine::SceneObject* obj_ptr : scene->get_all_objects()) {
                    if (obj_ptr->has_mesh()) {
                        const auto& mesh = obj_ptr->get_mesh_buffers();
                        for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
                            glm::vec3 v0(mesh.vertices[mesh.indices[i]*3], mesh.vertices[mesh.indices[i]*3+1], mesh.vertices[mesh.indices[i]*3+2]); glm::vec3 v1(mesh.vertices[mesh.indices[i+1]*3], mesh.vertices[mesh.indices[i+1]*3+1], mesh.vertices[mesh.indices[i+1]*3+2]); glm::vec3 v2(mesh.vertices[mesh.indices[i+2]*3], mesh.vertices[mesh.indices[i+2]*3+1], mesh.vertices[mesh.indices[i+2]*3+2]); float t; if (SnappingSystem::RayTriangleIntersect(rayOrigin, rayDir, v0, v1, v2, t) && t > 0 && t < closestHitDist) { closestHitDist = t; currentHoveredObjId = obj_ptr->get_id(); currentHoveredTriangleIdx = i; }
                        }
                    }
                }
                if (currentHoveredObjId != 0) {
                    Urbaxio::Engine::SceneObject* hitObject = scene->get_object_by_id(currentHoveredObjId);
                    if (hitObject) {
                        hoveredFaceTriangleIndices = FindCoplanarAdjacentTriangles(*hitObject, currentHoveredTriangleIdx);
                        hoveredObjId = currentHoveredObjId;
                    }
                }
            }
            else if (isDrawingLineMode) {
                // Drawing logic
                currentSnap = snappingSystem.FindSnapPoint(mouseX, mouseY, display_w, display_h, camera, *scene);
                glm::vec3 cursorWorldPoint = currentSnap.snapped ? currentSnap.worldPoint : GetCursorPointInWorld(camera, mouseX, mouseY, display_w, display_h, isPlacingSecondPoint ? currentLineStartPoint : glm::vec3(0.0f));
                if (isPlacingSecondPoint) {
                    glm::mat4 view = camera.GetViewMatrix(); glm::mat4 proj = camera.GetProjectionMatrix((display_h > 0) ? ((float)display_w / (float)display_h) : 1.0f);
                    if (!shiftDown && isAxisLocked) { isAxisLocked = false; }
                    else if (shiftPressedDownThisFrame && !isAxisLocked) { glm::vec2 startScreenPos, endScreenPos; bool sVis = SnappingSystem::WorldToScreen(currentLineStartPoint, view, proj, display_w, display_h, startScreenPos); bool eVis = SnappingSystem::WorldToScreen(cursorWorldPoint, view, proj, display_w, display_h, endScreenPos); if (sVis && eVis && glm::length2(endScreenPos - startScreenPos) > SCREEN_VECTOR_MIN_LENGTH_SQ) { glm::vec2 rbDir = glm::normalize(endScreenPos - startScreenPos); float maxDot = -1.0f; SnapType bestAxis = SnapType::NONE; glm::vec3 bestDir; const std::vector<std::pair<SnapType, glm::vec3>> axes = {{SnapType::AXIS_X, AXIS_X_DIR}, {SnapType::AXIS_Y, AXIS_Y_DIR}, {SnapType::AXIS_Z, AXIS_Z_DIR}}; glm::vec2 oScreen; if(SnappingSystem::WorldToScreen(glm::vec3(0.0f), view, proj, display_w, display_h, oScreen)) { for(const auto& ax : axes) { glm::vec2 axScreen; if(SnappingSystem::WorldToScreen(ax.second, view, proj, display_w, display_h, axScreen)) { glm::vec2 axDir = glm::normalize(axScreen - oScreen); float d = abs(glm::dot(rbDir, axDir)); if (d > maxDot) { maxDot = d; bestAxis = ax.first; bestDir = ax.second;}}}} if(bestAxis != SnapType::NONE) { isAxisLocked = true; lockedAxisType = bestAxis; lockedAxisDir = bestDir; } } }
                }
                if (isAxisLocked) { glm::vec3 basePt = ClosestPointOnLine(currentLineStartPoint, lockedAxisDir, cursorWorldPoint); SnapResult cSnap = snappingSystem.FindSnapPoint(mouseX, mouseY, display_w, display_h, camera, *scene); if (cSnap.snapped && IsProjectablePointSnap(cSnap.type)) { currentSnap.worldPoint = ClosestPointOnLine(currentLineStartPoint, lockedAxisDir, cSnap.worldPoint); currentSnap.snapped = true; currentSnap.type = cSnap.type; } else { currentSnap.worldPoint = basePt; currentSnap.snapped = true; currentSnap.type = lockedAxisType; }
                }
                currentRubberBandEnd = currentSnap.snapped ? currentSnap.worldPoint : cursorWorldPoint;
            }
        }
        
        if (enterPressedThisFrame && scene) {
            if (isPushPullActive) {
                float dist;
                auto [ptr, ec] = std::from_chars(lineLengthInputBuf, lineLengthInputBuf + strlen(lineLengthInputBuf), dist);
                if (ec == std::errc() && ptr == lineLengthInputBuf + strlen(lineLengthInputBuf)) {
                    scene->ExtrudeFace(pushPull_objId, pushPull_faceIndices, pushPull_faceNormal, dist);
                    isPushPullActive = false;
                    pushPullCurrentDistance = 0.0f;
                    lineLengthInputBuf[0] = '\0';
                }
            }
            else if (isPlacingSecondPoint) {
                 float length; auto [ptr, ec] = std::from_chars(lineLengthInputBuf, lineLengthInputBuf + strlen(lineLengthInputBuf), length); if (ec == std::errc() && ptr == lineLengthInputBuf + strlen(lineLengthInputBuf) && length > 1e-4f) { lineLengthValue = length; glm::vec3 direction; if (isAxisLocked) { float dotProd = glm::dot(currentRubberBandEnd - currentLineStartPoint, lockedAxisDir); direction = (dotProd >= 0.0f) ? lockedAxisDir : -lockedAxisDir; } else { direction = currentRubberBandEnd - currentLineStartPoint; if (glm::length(direction) < 1e-6f) { isPlacingSecondPoint = false; isPlacingFirstPoint = true; isAxisLocked = false; lineLengthInputBuf[0] = '\0'; goto end_event_processing_final_label; } direction = glm::normalize(direction); } glm::vec3 finalEndPoint = currentLineStartPoint + direction * lineLengthValue; scene->AddUserLine(currentLineStartPoint, finalEndPoint); isPlacingSecondPoint = false; isPlacingFirstPoint = true; isAxisLocked = false; lineLengthInputBuf[0] = '\0'; } else { lineLengthInputBuf[0] = '\0'; }
            }
        }
        end_event_processing_final_label:;

        HandleMouseMotion(camera, window, display_w, display_h);
    }

    void InputHandler::HandleMouseMotion(Urbaxio::Camera& camera, SDL_Window* window, int display_w, int display_h) { /* ... */ int cX, cY; SDL_GetMouseState(&cX, &cY); if (middleMouseButtonDown && isMouseFocused) { if (firstMouse) { lastMouseX = cX; lastMouseY = cY; firstMouse = false; } else { float dX = static_cast<float>(cX - lastMouseX); float dY = static_cast<float>(cY - lastMouseY); if (std::abs(dX) > 1e-3f || std::abs(dY) > 1e-3f) { if (shiftDown) { camera.ProcessPan(dX, dY); } else { camera.ProcessOrbit(dX, dY); } int nX = cX; int nY = cY; bool w = false; const int m = 1; if (display_w > (m + 1) * 2 && display_h > (m + 1) * 2) { if (cX <= m) { nX = display_w - (m + 2); w = true; } else if (cX >= display_w - (m + 1)) { nX = m + 1; w = true; } if (cY <= m) { nY = display_h - (m + 2); w = true; } else if (cY >= display_h - (m + 1)) { nY = m + 1; w = true; } } if (w) { SDL_WarpMouseInWindow(window, nX, nY); lastMouseX = nX; lastMouseY = nY; } else { lastMouseX = cX; lastMouseY = cY; } } else { lastMouseX = cX; lastMouseY = cY; } } } else { firstMouse = true; lastMouseX = cX; lastMouseY = cY; } }

} // namespace Urbaxio