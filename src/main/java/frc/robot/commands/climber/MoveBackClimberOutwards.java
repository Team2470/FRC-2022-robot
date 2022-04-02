// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BackClimber;
import frc.robot.subsystems.Climber;

public class MoveBackClimberOutwards extends CommandBase {

  private final BackClimber m_climber;
  /** Creates a new ForwardClimbClockwise. */
  public MoveBackClimberOutwards(BackClimber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;

    addRequirements(m_climber);
  }

  @Override
  public void initialize() {
    m_climber.disable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.startOutwardClimb();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stop();
  }
}
