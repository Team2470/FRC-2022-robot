// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class DriveWithGamepadCommand extends CommandBase {
  private final SendableChooser<String> m_joystickLayoutChooser;
  private final Drive m_drive;
  private final XboxController m_controller;
  /** Creates a new DriveWithGamepadCommand. */
  public DriveWithGamepadCommand(Drive drive, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_controller = controller;

    m_joystickLayoutChooser = new SendableChooser<>();
    m_joystickLayoutChooser.setDefaultOption("Curvature", "Curvature");
    m_joystickLayoutChooser.addOption("Arcade", "Arcade");
    SmartDashboard.putData("Drive Joystick layout", m_joystickLayoutChooser);

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch(m_joystickLayoutChooser.getSelected()) {
      case "Curvature":
        double xSpeed = -m_controller.getLeftY();
        double zRotation = m_controller.getRightX();
        boolean quickTurn = m_controller.getLeftBumper();

        m_drive.curvatureDrive(xSpeed, zRotation, quickTurn);
        break;
      case "Arcade":
        m_drive.arcadeDrive(-m_controller.getLeftY(),m_controller.getRightX());
        break;
      default:
        m_drive.stop();
    }

  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }
    
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
