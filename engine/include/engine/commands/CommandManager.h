#pragma once

#include "engine/commands/ICommand.h"
#include <vector>
#include <memory>

namespace Urbaxio::Engine {

class CommandManager {
public:
    CommandManager();

    // Executes a command and adds it to the undo stack
    void ExecuteCommand(std::unique_ptr<ICommand> command);

    // Undoes the last command
    void Undo();

    // Redoes the last undone command
    void Redo();

    // Checks if there are commands to undo/redo
    bool HasUndo() const;
    bool HasRedo() const;
    
    // Clears the history, e.g., for a new scene
    void ClearHistory();

private:
    std::vector<std::unique_ptr<ICommand>> undoStack_;
    std::vector<std::unique_ptr<ICommand>> redoStack_;
};

} // namespace Urbaxio::Engine 