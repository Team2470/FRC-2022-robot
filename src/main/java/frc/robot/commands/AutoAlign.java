// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class AutoAlign extends CommandBase {
  private final Vision m_vision;
  private final Drive m_drive;
  private final double m_kp = 0.005;
  private final Rotation2d m_minimum = Rotation2d.fromDegrees(0.29);

  /**
   * Creates a new DriveWithGamepadCommand.
   */
  public AutoAlign(Vision vision, Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_vision = vision;
    m_drive = drive;

    addRequirements(drive);
  }

  private Rotation2d getAngleAdjust(Rotation2d angle) {
    if (angle.getDegrees() > 0) {
      return angle.times(m_kp).plus(m_minimum);
    } else if (angle.getDegrees() < 0) {
      return angle.times(m_kp).minus(m_minimum);

    }
    return Rotation2d.fromDegrees(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_vision.getTargetFound()) {
      Rotation2d angle = m_vision.getHorizontalAngle();
      Rotation2d angleOutput = getAngleAdjust(angle);
      
      m_drive.arcadeDrive(0, angleOutput.getDegrees());
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return Math.abs(m_vision.getHorizontalAngle().getDegrees()) < 2;

  }

}
