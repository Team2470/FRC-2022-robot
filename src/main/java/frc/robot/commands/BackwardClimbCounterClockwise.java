// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.BackClimber;
import frc.robot.subsystems.FrontClimber;

public class BackwardClimbCounterClockwise extends CommandBase {

  private BackClimber m_backClimber; 
  /** Creates a new ForwardClimbClockwise. */
  public BackwardClimbCounterClockwise(BackClimber frontClimber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_backClimber = m_backClimber;

    addRequirements(m_backClimber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Backward");

    m_backClimber.startClimbMotor(Constants.kCounterClockwise, Constants.kClimberSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_backClimber.stopClimbMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
