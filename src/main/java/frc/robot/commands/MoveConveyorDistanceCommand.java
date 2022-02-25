// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class MoveConveyorDistanceCommand extends CommandBase {
  /** Creates a new MoveConveyerDistanceCommand. */
  public MoveConveyorDistanceCommand(Conveyor conveyor, double inches) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(conveyor);
  }

  @Override
  public void execute() {
    // TODO: Write out stub
  }

  @Override
  public void end(boolean interrupted) {
    // TODO: Write out stub
  }

  @Override
  public boolean isFinished() {
    // TODO: Write out stub
    return true;
  }
}
