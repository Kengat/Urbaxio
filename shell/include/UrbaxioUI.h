#pragma once
// -----------------------------------------------------------------------------
// Urbaxio native UI layer powered by RmlUi (HTML/CSS).
// Lets the whole interface be styled freely with .rml/.rcss while rendering
// through the app's own OpenGL context, in-process, on top of the 3D scene.
// -----------------------------------------------------------------------------
#include <SDL2/SDL.h>
#include <functional>
#include <string>

namespace Urbaxio::UI {

// Initialise RmlUi: render/system interfaces, context, fonts, and load the
// main document. Requires an active OpenGL 3.3 context. The window is used for
// custom title-bar dragging of the borderless window. Returns false on failure.
bool InitRml(SDL_Window* window, int width, int height);

// Release the document, context, interfaces and shut RmlUi down.
void ShutdownRml();

// Callback invoked when the UI fires an action (e.g. data-event-click="action('exit')").
void SetActionCallback(std::function<void(const std::string&)> cb);

// Forward one SDL event to the UI. Returns true if the pointer is currently over
// a UI element (so the caller can suppress 3D-tool/camera interaction).
bool ProcessRmlEvent(const SDL_Event& event);

// True if the mouse currently hovers a UI element.
bool IsMouseOverUI();

// Advance UI animations/layout for the frame.
void UpdateRml();

// Draw the UI. Call after the scene (and after ImGui) and before SwapWindow.
void RenderRml(int width, int height);

} // namespace Urbaxio::UI
