// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Shooter;

public class AutoAlign extends CommandBase {
  private final Vision m_vision;
  private final Shooter m_shooter;
  private final Drive m_drive;
  private final double m_kp = 0.01;
  private final Rotation2d m_minimum = Rotation2d.fromDegrees(0.29);
  /** Creates a new DriveWithGamepadCommand. */
  public AutoAlign(Vision vision, Shooter shooter, Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_vision = vision;
    m_shooter = shooter;
    m_drive = drive;


    addRequirements(vision);
    addRequirements(shooter);
    addRequirements(drive);
  }

  private Rotation2d getAngleAdjust(Rotation2d angle){
    Rotation2d turnAngle = Rotation2d.fromDegrees(0);
    if(angle.getDegrees() > 0) {
      turnAngle = angle.times(m_kp).plus(m_minimum);
    }
    else if (angle.getDegrees() < 0) {
      turnAngle = angle.times(m_kp).minus(m_minimum);

    }
    return turnAngle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_vision.getTargetFound())  {
      Rotation2d angle = m_vision.getHorizontalAngle();
      Rotation2d angleOutput = getAngleAdjust(angle);

      double distance = m_vision.geTargetDistanceM();
      

     m_drive.arcadeDrive(0, angleOutput.getDegrees());}

  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    boolean targetAligned = Math.abs(m_vision.getHorizontalAngle().getDegrees()) < 0.2;
    return targetAligned;

  }

}
