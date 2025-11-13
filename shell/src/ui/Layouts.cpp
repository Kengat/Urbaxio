#include "ui/Layouts.h"

#include "ui/IVRWidget.h"

#include <numeric>

namespace Urbaxio::UI {

// --- VerticalLayout Implementation ---

void VerticalLayout::Apply(std::vector<std::unique_ptr<IVRWidget>>& widgets, const glm::vec2& panelSize) {
    if (widgets.empty()) return;

    if (stretch_) {
        float totalSpacing = (widgets.size() - 1) * spacing_;
        float availableHeight = panelSize.y - totalSpacing;
        float widgetHeight = (widgets.size() > 0) ? (availableHeight / widgets.size()) : 0.0f;

        float currentY = panelSize.y / 2.0f;
        for (auto& widget : widgets) {
            currentY -= widgetHeight / 2.0f;
            widget->SetLocalPosition({0.0f, currentY, 0.01f});
            widget->SetSize({panelSize.x, widgetHeight});
            currentY -= (widgetHeight / 2.0f + spacing_);
        }
    } else {
        float totalHeight = 0;
        for (const auto& widget : widgets) {
            totalHeight += widget->GetSize().y;
        }
        totalHeight += (widgets.size() - 1) * spacing_;

        float currentY = totalHeight / 2.0f;
        for (auto& widget : widgets) {
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

    

    // Calculate how many buttons can fit with FIXED size

    int maxColumns = std::max(1, static_cast<int>((panelSize.x + spacing_.x) / (buttonSize_ + spacing_.x)));

    

    // If panel is narrow, force single column

    if (panelSize.x < buttonSize_ * 1.5f) {

        maxColumns = 1;

    }

    

    int columns = maxColumns;

    int rows = (widgets.size() + columns - 1) / columns;

    

    // DO NOT change button size - keep it fixed!

    float cellWidth = buttonSize_;

    float cellHeight = buttonSize_;

    

    // Center the grid in the panel

    float totalWidth = columns * cellWidth + (columns - 1) * spacing_.x;

    float startX = -totalWidth / 2.0f + cellWidth / 2.0f;

    float startY = panelSize.y / 2.0f - cellHeight / 2.0f;

    

    for (size_t i = 0; i < widgets.size(); ++i) {

        int row = i / columns;

        int col = i % columns;

        

        float x = startX + col * (cellWidth + spacing_.x);

        float y = startY - row * (cellHeight + spacing_.y);

        

        widgets[i]->SetLocalPosition({x, y, 0.01f});

        // DO NOT call SetSize - keep original button size!

    }

}

}

