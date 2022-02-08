// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.BackwardClimbClockwise;
import frc.robot.commands.BackwardClimbCounterClockwise;
import frc.robot.commands.DriveWithGamepadCommand;
import frc.robot.commands.ForwardClimbClockwise;
import frc.robot.commands.ForwardClimbCounterClockwise;
import frc.robot.subsystems.BackClimber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.FrontClimber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive m_drive = new Drive();

  private final FrontClimber m_frontClimber = new FrontClimber();

  private final BackClimber m_backClimber = new BackClimber();

  private final XboxController m_controller = new XboxController(Constants.kControllerA);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_drive.setDefaultCommand(new DriveWithGamepadCommand(m_drive,m_controller));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    JoystickButton ForwardClimbClockwiseButton = new JoystickButton(m_controller, XboxController.Button.kY.value);
    ForwardClimbClockwiseButton.whileHeld(new ForwardClimbClockwise(m_frontClimber));

    JoystickButton ForwardClimbCounterClockwiseButton = new JoystickButton(m_controller, XboxController.Button.kB.value);
    ForwardClimbCounterClockwiseButton.whileHeld(new ForwardClimbCounterClockwise(m_frontClimber));

    JoystickButton BackwardClimbClockwiseButton = new JoystickButton(m_controller, XboxController.Button.kX.value);
    BackwardClimbClockwiseButton.whileHeld(new BackwardClimbClockwise(m_backClimber));

    JoystickButton BackwardClimbCounterClockwiseButton = new JoystickButton(m_controller, XboxController.Button.kA.value);
    BackwardClimbCounterClockwiseButton.whileHeld(new BackwardClimbCounterClockwise(m_backClimber));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Return null for no autonomous command
    return null;
  }
}
