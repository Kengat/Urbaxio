#include "engine/commands/CommandManager.h"
#include <iostream>

namespace Urbaxio::Engine {

CommandManager::CommandManager() {}

void CommandManager::ExecuteCommand(std::unique_ptr<ICommand> command) {
    if (!command) {
        return;
    }
    const char* commandName = command->GetName();
    std::cout << "CommandManager: Executing command '" << (commandName ? commandName : "Unknown") << "'" << std::endl;
    
    // Execute the command. The command itself is responsible for changing the model.
    command->Execute();
    
    // Move the executed command to the undo stack.
    undoStack_.push_back(std::move(command));

    // A new action invalidates the old redo history.
    if (!redoStack_.empty()) {
        std::cout << "CommandManager: Clearing redo stack (" << redoStack_.size() << " commands)." << std::endl;
        redoStack_.clear();
    }
}

void CommandManager::Undo() {
    if (undoStack_.empty()) {
        std::cout << "CommandManager: Nothing to undo." << std::endl;
        return;
    }

    // Move the command from the undo stack to the redo stack.
    std::unique_ptr<ICommand> commandToUndo = std::move(undoStack_.back());
    undoStack_.pop_back();

    const char* commandName = commandToUndo->GetName();
    std::cout << "CommandManager: Undoing command '" << (commandName ? commandName : "Unknown") << "'" << std::endl;

    // Perform the undo operation.
    commandToUndo->Undo();

    redoStack_.push_back(std::move(commandToUndo));
}

void CommandManager::Redo() {
    if (redoStack_.empty()) {
        std::cout << "CommandManager: Nothing to redo." << std::endl;
        return;
    }

    // Move the command from the redo stack back to the undo stack.
    std::unique_ptr<ICommand> commandToRedo = std::move(redoStack_.back());
    redoStack_.pop_back();

    const char* commandName = commandToRedo->GetName();
    std::cout << "CommandManager: Redoing command '" << (commandName ? commandName : "Unknown") << "'" << std::endl;

    // Re-execute the command.
    commandToRedo->Execute();

    undoStack_.push_back(std::move(commandToRedo));
}

bool CommandManager::HasUndo() const {
    return !undoStack_.empty();
}

bool CommandManager::HasRedo() const {
    return !redoStack_.empty();
}

void CommandManager::ClearHistory() {
    undoStack_.clear();
    redoStack_.clear();
    std::cout << "CommandManager: History cleared." << std::endl;
}

} // namespace Urbaxio::Engine 