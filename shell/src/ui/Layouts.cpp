#include "ui/Layouts.h"

#include "ui/IVRWidget.h"

#include <numeric>

namespace Urbaxio::UI {

// --- VerticalLayout Implementation ---

void VerticalLayout::Apply(std::vector<std::unique_ptr<IVRWidget>>& widgets, const glm::vec2& panelSize) {
    if (widgets.empty()) return;

    // --- NEW: Filter visible widgets ---
    std::vector<IVRWidget*> visibleWidgets;
    for (auto& w : widgets) {
        if (w->IsVisible()) visibleWidgets.push_back(w.get());
    }
    if (visibleWidgets.empty()) return;
    // -----------------------------------

    if (stretch_) {
        float totalSpacing = (visibleWidgets.size() - 1) * spacing_;
        float availableHeight = panelSize.y - totalSpacing;
        float widgetHeight = (visibleWidgets.size() > 0) ? (availableHeight / visibleWidgets.size()) : 0.0f;

        float currentY = panelSize.y / 2.0f;
        for (auto* widget : visibleWidgets) {
            currentY -= widgetHeight / 2.0f;
            widget->SetLocalPosition({0.0f, currentY, 0.01f});
            widget->SetSize({panelSize.x, widgetHeight});
            currentY -= (widgetHeight / 2.0f + spacing_);
        }
    } else {
        float totalHeight = 0;
        for (const auto* widget : visibleWidgets) {
            totalHeight += widget->GetSize().y;
        }
        totalHeight += (visibleWidgets.size() - 1) * spacing_;

        float currentY = totalHeight / 2.0f;
        for (auto* widget : visibleWidgets) {
            const auto& widgetSize = widget->GetSize();
            currentY -= widgetSize.y / 2.0f;
            widget->SetLocalPosition({0.0f, currentY, 0.01f}); // Center horizontally
            currentY -= (widgetSize.y / 2.0f + spacing_);
        }
    }
}

// --- GridLayout Implementation ---

void GridLayout::Apply(std::vector<std::unique_ptr<IVRWidget>>& widgets, const glm::vec2& panelSize) {

    if (widgets.empty() || columns_ <= 0) return;

    int numRows = (widgets.size() + columns_ - 1) / columns_;

    float cellWidth = (panelSize.x - (columns_ - 1) * spacing_.x) / columns_;

    float cellHeight = (panelSize.y - (numRows - 1) * spacing_.y) / numRows;

    float startX = -panelSize.x / 2.0f + cellWidth / 2.0f;

    float startY = panelSize.y / 2.0f - cellHeight / 2.0f;

    for (size_t i = 0; i < widgets.size(); ++i) {

        int row = i / columns_;

        int col = i % columns_;

        float x = startX + col * (cellWidth + spacing_.x);

        float y = startY - row * (cellHeight + spacing_.y);

        widgets[i]->SetLocalPosition({x, y, 0.01f});

    }

}

// --- AdaptiveGridLayout Implementation ---

void AdaptiveGridLayout::Apply(std::vector<std::unique_ptr<IVRWidget>>& widgets, const glm::vec2& panelSize) {

    if (widgets.empty()) return;

    // --- NEW: Filter visible widgets ---
    std::vector<IVRWidget*> visibleWidgets;
    for (auto& w : widgets) {
        if (w->IsVisible()) visibleWidgets.push_back(w.get());
    }
    if (visibleWidgets.empty()) return;
    // -----------------------------------

    bool isHorizontal = (panelSize.x > panelSize.y);

    

    int columns;

    int rows;

    

    if (isHorizontal) {

        // Horizontal panel - check if we should use single-row mode

        int maxRowsThatFit = std::max(1, static_cast<int>((panelSize.y + spacing_.y) / (buttonSize_ + spacing_.y)));

        

        // FIX: Only use single-row when REALLY narrow (only 1 row fits)

        if (maxRowsThatFit == 1) {

            // SINGLE-ROW MODE: all widgets in one horizontal line

            columns = visibleWidgets.size();

            rows = 1;

        } else {

            // GRID MODE

            int maxColumns = std::max(1, static_cast<int>((panelSize.x + spacing_.x) / (buttonSize_ + spacing_.x)));

            if (panelSize.x < buttonSize_ * 1.5f) {

                maxColumns = 1;

            }

            columns = maxColumns;

            rows = (visibleWidgets.size() + columns - 1) / columns;

        }

    } else {

        // Vertical panel - use normal grid logic

        int maxColumns = std::max(1, static_cast<int>((panelSize.x + spacing_.x) / (buttonSize_ + spacing_.x)));

        if (panelSize.x < buttonSize_ * 1.5f) {

            maxColumns = 1;

        }

        columns = maxColumns;

        rows = (visibleWidgets.size() + columns - 1) / columns;

    }

    

    // Fixed button size

    float cellWidth = buttonSize_;

    float cellHeight = buttonSize_;

    

    // Calculate total content size

    float totalContentWidth = columns * cellWidth + (columns - 1) * spacing_.x;

    float totalContentHeight = rows * cellHeight + (rows - 1) * spacing_.y;

    

    // FIX: Center content relative to ITS OWN center (0, 0), not panel center

    float startX = -totalContentWidth / 2.0f + cellWidth / 2.0f;

    float startY = totalContentHeight / 2.0f - cellHeight / 2.0f;

    

    for (size_t i = 0; i < visibleWidgets.size(); ++i) {

        int row = i / columns;

        int col = i % columns;

        

        float x = startX + col * (cellWidth + spacing_.x);

        float y = startY - row * (cellHeight + spacing_.y);

        

        visibleWidgets[i]->SetLocalPosition({x, y, 0.01f});

    }

}

}

