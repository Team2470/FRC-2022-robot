// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.BackClimber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.FrontClimber;
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
  private final Conveyor m_conveyor = new Conveyor();
  private final Shooter m_shooter = new Shooter();

  private final FrontClimber m_frontClimber = new FrontClimber();

  private final BackClimber m_backClimber = new BackClimber();

  private final XboxController m_controller = new XboxController(Constants.kControllerA);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_drive.setDefaultCommand(new DriveWithGamepadCommand(m_drive,m_controller));
    JoystickButton conveyorUp = new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);
    conveyorUp.whileHeld(new RunConveyorCommand(m_conveyor, RunConveyorCommand.Direction.kUp));

    JoystickButton conveyorDown = new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);
    conveyorDown.whileHeld(new RunConveyorCommand(m_conveyor, RunConveyorCommand.Direction.kDown));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    JoystickButton rpmButton1 = new JoystickButton(m_controller, XboxController.Button.kA.value);
    rpmButton1.whileHeld(new RunShooterCommand(m_shooter, 1500));

    JoystickButton rpmButton2 = new JoystickButton(m_controller, XboxController.Button.kX.value);
    rpmButton2.whileHeld(new RunShooterCommand(m_shooter, 2000));

    JoystickButton rpmButton3 = new JoystickButton(m_controller, XboxController.Button.kB.value);
    rpmButton3.whileHeld(new RunShooterCommand(m_shooter, 2500));

    /*JoystickButton ForwardClimbClockwiseButton = new JoystickButton(m_controller, XboxController.Button.kY.value);
    ForwardClimbClockwiseButton.whileHeld(new ForwardClimbClockwise(m_frontClimber));

    JoystickButton ForwardClimbCounterClockwiseButton = new JoystickButton(m_controller, XboxController.Button.kB.value);
    ForwardClimbCounterClockwiseButton.whileHeld(new ForwardClimbCounterClockwise(m_frontClimber));

    JoystickButton BackwardClimbClockwiseButton = new JoystickButton(m_controller, XboxController.Button.kX.value);
    BackwardClimbClockwiseButton.whileHeld(new BackwardClimbClockwise(m_backClimber));

    JoystickButton BackwardClimbCounterClockwiseButton = new JoystickButton(m_controller, XboxController.Button.kA.value);
    BackwardClimbCounterClockwiseButton.whileHeld(new BackwardClimbCounterClockwise(m_backClimber));*/
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Return null for no autonomous command
    return new ParallelCommandGroup(
            new RunShooterCommand(m_shooter, 2500),
            new SequentialCommandGroup(
                    new WaitCommand(2),
                    new RunConveyorCommand(m_conveyor, RunConveyorCommand.Direction.kUp).withTimeout(5)
            )
    );
  }
}
