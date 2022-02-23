// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class RunShooterCommand extends CommandBase {
  private final Shooter m_shooter;
  private final double m_setPoint;
  /** Creates a new RunShooterCommand. */
  public RunShooterCommand(Shooter shooter, double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_setPoint = setPoint;


    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setRPM(m_setPoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
  }

}
