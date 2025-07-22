#define GLM_ENABLE_EXPERIMENTAL
#include "input_handler.h"
#include "camera.h"
#include "snapping.h"
#include <tools/ToolManager.h>

#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <SDL2/SDL.h>
#include <iostream>
#include <cmath>

namespace Urbaxio {

    InputHandler::InputHandler() 
        : middleMouseButtonDown(false), shiftDown(false), ctrlDown(false),
          lastMouseX(0), lastMouseY(0), isMouseFocused(true), firstMouse(true) {}
    
    glm::vec3 InputHandler::GetCursorPointInWorld(const Camera& camera, int mouseX, int mouseY, int screenWidth, int screenHeight, const glm::vec3& fallbackPlanePoint) { 
        glm::vec3 point; 
        if (Urbaxio::SnappingSystem::RaycastToZPlane(mouseX, mouseY, screenWidth, screenHeight, camera, point)) { 
            return point; 
        } else { 
            glm::vec3 rayOrigin, rayDirection; 
            Camera::ScreenToWorldRay(mouseX, mouseY, screenWidth, screenHeight, camera.GetViewMatrix(), camera.GetProjectionMatrix((screenHeight > 0) ? ((float)screenWidth / (float)screenHeight) : 1.0f), rayOrigin, rayDirection); 
            glm::vec3 planeNormal = -camera.Front; 
            glm::vec3 pointOnPlane = fallbackPlanePoint; 
            float denom = glm::dot(rayDirection, planeNormal); 
            if (std::abs(denom) > 1e-6f) { 
                float t = glm::dot(pointOnPlane - rayOrigin, planeNormal) / denom; 
                if (t > 1e-4f && t < 10000.0f) return rayOrigin + rayDirection * t; 
            } 
            return rayOrigin + rayDirection * 10.0f; // Fallback
        } 
    }

    void InputHandler::ProcessEvents(
        Urbaxio::Camera& camera,
        bool& should_quit,
        SDL_Window* window,
        Tools::ToolManager& toolManager,
        Urbaxio::Engine::Scene* scene // <-- FIX: Added missing parameter to match declaration
    ) {
        SDL_Event event; 
        ImGuiIO& io = ImGui::GetIO();

        // Update modifier key states at the start of the frame
        const Uint8* keyboardState = SDL_GetKeyboardState(NULL);
        shiftDown = keyboardState[SDL_SCANCODE_LSHIFT] || keyboardState[SDL_SCANCODE_RSHIFT];
        ctrlDown = keyboardState[SDL_SCANCODE_LCTRL] || keyboardState[SDL_SCANCODE_RCTRL] || keyboardState[SDL_SCANCODE_LGUI] || keyboardState[SDL_SCANCODE_RGUI];

        while (SDL_PollEvent(&event)) { 
            ImGui_ImplSDL2_ProcessEvent(&event); 
            bool wantCaptureMouse = io.WantCaptureMouse; 
            bool wantCaptureKeyboard = io.WantCaptureKeyboard; 
            
            switch (event.type) { 
                case SDL_QUIT: 
                    should_quit = true; 
                    break;
                
                case SDL_WINDOWEVENT: {
                    int w, h;
                    SDL_GetWindowSize(window, &w, &h);
                    switch (event.window.event) {
                        case SDL_WINDOWEVENT_FOCUS_GAINED: isMouseFocused = true; break;
                        case SDL_WINDOWEVENT_FOCUS_LOST:
                            isMouseFocused = false;
                            middleMouseButtonDown = false;
                            SDL_ShowCursor(SDL_ENABLE);
                            toolManager.SetTool(Tools::ToolType::Select); // Revert to select tool on focus loss
                            break;
                        case SDL_WINDOWEVENT_ENTER: isMouseFocused = true; break;
                        case SDL_WINDOWEVENT_LEAVE: isMouseFocused = false; break;
                    }
                    break;
                }
                
                case SDL_KEYDOWN:
                    // DO NOT update modifier states from event.key.keysym.mod here.
                    // The keyboardState check at the top of ProcessEvents is more reliable.
                    if (!wantCaptureKeyboard) {
                        // --- Undo/Redo ---
                        if (ctrlDown && event.key.keysym.sym == SDLK_z) {
                            scene->getCommandManager()->Undo();
                        } else if (ctrlDown && event.key.keysym.sym == SDLK_y) {
                            scene->getCommandManager()->Redo();
                        } else {
                            // Forward other key presses to the active tool
                            toolManager.OnKeyDown(event.key.keysym.sym, shiftDown, ctrlDown);
                        }
                    }
                    break;

                case SDL_KEYUP:
                    // We don't need to do anything here anymore, the state will be re-evaluated
                    // at the start of the next frame.
                    break;
                
                case SDL_MOUSEBUTTONDOWN:
                    if (!wantCaptureMouse) {
                        if (event.button.button == SDL_BUTTON_MIDDLE) {
                             middleMouseButtonDown = true;
                             firstMouse = true; 
                             SDL_ShowCursor(SDL_DISABLE);
                        } else if (event.button.button == SDL_BUTTON_LEFT) {
                            toolManager.OnLeftMouseDown(event.button.x, event.button.y, shiftDown, ctrlDown);
                        } else if (event.button.button == SDL_BUTTON_RIGHT) {
                            toolManager.OnRightMouseDown();
                        }
                    } 
                    break;
                
                case SDL_MOUSEBUTTONUP: 
                    if (event.button.button == SDL_BUTTON_MIDDLE) {
                         middleMouseButtonDown = false;
                         SDL_ShowCursor(SDL_ENABLE); 
                    } else if (event.button.button == SDL_BUTTON_LEFT && !wantCaptureMouse) {
                         toolManager.OnLeftMouseUp(event.button.x, event.button.y, shiftDown, ctrlDown);
                    }
                    break;
                
                case SDL_MOUSEWHEEL: 
                    if (!wantCaptureMouse) { 
                        camera.ProcessMouseScroll(static_cast<float>(event.wheel.y));
                    } 
                    break;

                case SDL_MOUSEMOTION:
                    toolManager.OnMouseMove(event.motion.x, event.motion.y);
                    break;
            }
        }

        // Handle continuous actions like camera panning/orbiting
        int w, h;
        SDL_GetWindowSize(window, &w, &h);
        HandleMouseMotion(camera, window, w, h);
    }

    void InputHandler::HandleMouseMotion(Urbaxio::Camera& camera, SDL_Window* window, int display_w, int display_h) {
        int cX, cY;
        SDL_GetMouseState(&cX, &cY);

        if (middleMouseButtonDown && isMouseFocused) {
            if (firstMouse) {
                lastMouseX = cX;
                lastMouseY = cY;
                firstMouse = false;
            } else {
                float dX = static_cast<float>(cX - lastMouseX);
                float dY = static_cast<float>(cY - lastMouseY);
                if (std::abs(dX) > 1e-3f || std::abs(dY) > 1e-3f) {
                    if (shiftDown) {
                        camera.ProcessPan(dX, dY);
                    } else {
                        camera.ProcessOrbit(dX, dY);
                    }

                    // Warp mouse to keep it in the window for continuous movement
                    int nX = cX, nY = cY;
                    bool warped = false;
                    const int margin = 1;
                    if (display_w > (margin + 1) * 2 && display_h > (margin + 1) * 2) {
                        if (cX <= margin) { nX = display_w - (margin + 2); warped = true; }
                        else if (cX >= display_w - (margin + 1)) { nX = margin + 1; warped = true; }
                        if (cY <= margin) { nY = display_h - (margin + 2); warped = true; }
                        else if (cY >= display_h - (margin + 1)) { nY = margin + 1; warped = true; }
                    }
                    if (warped) {
                        SDL_WarpMouseInWindow(window, nX, nY);
                        lastMouseX = nX;
                        lastMouseY = nY;
                    } else {
                        lastMouseX = cX;
                        lastMouseY = cY;
                    }
                } else {
                    lastMouseX = cX;
                    lastMouseY = cY;
                }
            }
        } else {
            firstMouse = true;
            lastMouseX = cX;
            lastMouseY = cY;
        }
    }

} // namespace Urbaxio