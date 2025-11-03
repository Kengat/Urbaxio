#pragma once

#include <glm/glm.hpp>

#include <vector>

#include <memory>

namespace Urbaxio::UI {

class IVRWidget; // Forward declaration

// Enum to define the type of layout

enum class LayoutType {

    NONE,

    VERTICAL,

    GRID

};

// Base interface for all layout managers

class ILayout {

public:

    virtual ~ILayout() = default;

    // The core function: takes widgets and panel size, then positions the widgets

    virtual void Apply(std::vector<std::unique_ptr<IVRWidget>>& widgets, const glm::vec2& panelSize) = 0;

};

// Lays out widgets in a single vertical column

class VerticalLayout : public ILayout {

public:
    VerticalLayout(float spacing = 0.01f, bool stretch = false) : spacing_(spacing), stretch_(stretch) {}

    void Apply(std::vector<std::unique_ptr<IVRWidget>>& widgets, const glm::vec2& panelSize) override;

private:
    float spacing_;
    bool stretch_;
};

// Lays out widgets in a grid

class GridLayout : public ILayout {

public:

    GridLayout(int columns, const glm::vec2& spacing = {0.01f, 0.01f}) : columns_(columns), spacing_(spacing) {}

    void Apply(std::vector<std::unique_ptr<IVRWidget>>& widgets, const glm::vec2& panelSize) override;

private:

    int columns_;

    glm::vec2 spacing_;

};

}

