// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
  private final double m_minimum = 0.29;
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

  private double getAngleAdjust(double angle){
    double turnAngle = 0;
    if(angle > 0) {
      turnAngle = m_kp * angle + m_minimum;
    }
    else if (angle < 0) {
      turnAngle = m_kp * angle - m_minimum;

    }
    return turnAngle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_vision.getTargetFound())  {
      double angle = m_vision.getHorizontalAngleD();
      double angleOutput = getAngleAdjust(angle);

      double distance = m_vision.geTargetDistanceM();
      

     m_drive.arcadeDrive(0, angleOutput);}

  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    boolean targetAligned = Math.abs(m_vision.getHorizontalAngleD()) < 0.2;
    return targetAligned;

  }

}
