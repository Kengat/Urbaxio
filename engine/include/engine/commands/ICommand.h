#pragma once

#include <memory>

namespace Urbaxio::Engine {

// Interface for all commands that modify the scene state.
// This is the core of the Command design pattern.
class ICommand {
public:
    virtual ~ICommand() = default;

    // Executes the command's action.
    virtual void Execute() = 0;

    // Reverts the command's action.
    virtual void Undo() = 0;

    // Returns the name of the command for UI or debugging purposes.
    virtual const char* GetName() const = 0;
};

} // namespace Urbaxio::Engine 