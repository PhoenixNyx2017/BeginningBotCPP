// Copyright (c) FRC Team 122. All Rights Reserved.

#include "commands/ExampleCommand.h"

ExampleCommand::ExampleCommand(ExampleSubsystem *subsystem)
    : m_subsystem{subsystem} {
  // Register that this command requires the subsystem.
  AddRequirements(m_subsystem);
}
