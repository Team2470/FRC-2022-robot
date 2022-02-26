// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;


public class RunConveyorCommand extends CommandBase {
  /**
   * Creates a new RunConveyorCommand.
   */
  private final Conveyor m_conveyor;
  private final Direction m_direction;

  public enum Direction {
    kUp,
    kDown
  }

  public RunConveyorCommand(Conveyor conveyor, Direction direction) {
    m_conveyor = conveyor;
    m_direction = direction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_conveyor);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_direction) {
      case kUp:
        m_conveyor.up();
      case kDown:
        m_conveyor.down();
      default:
        m_conveyor.up();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_conveyor.stop();
  }

}
