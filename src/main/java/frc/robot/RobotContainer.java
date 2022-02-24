// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kennedyrobotics.triggers.DPadTrigger;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Conveyor m_conveyor = new Conveyor();
  private final Drive m_drive = new Drive(m_conveyor.getMotor());
  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();
  private final FrontClimber m_frontClimber = new FrontClimber();
  private final BackClimber m_backClimber = new BackClimber();

  // Controller
  private final XboxController m_controller = new XboxController(Constants.kControllerA);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_drive.setDefaultCommand(new DriveWithGamepadCommand(m_drive,m_controller));
    m_intake.setDefaultCommand(new RetractIntakeCommand(m_intake));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //: Conveyor Control
    JoystickButton conveyorUp = new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);
    conveyorUp.whileHeld(new RunConveyorCommand(m_conveyor, RunConveyorCommand.Direction.kUp));

    JoystickButton conveyorDown = new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);
    conveyorDown.whileHeld(new RunConveyorCommand(m_conveyor, RunConveyorCommand.Direction.kDown));
    //: Shooter control
    JoystickButton rpmButton1 = new JoystickButton(m_controller, XboxController.Button.kA.value);
    rpmButton1.whileHeld(new RunShooterCommand(m_shooter, 1500));

    JoystickButton rpmButton2 = new JoystickButton(m_controller, XboxController.Button.kX.value);
    rpmButton2.whileHeld(new RunShooterCommand(m_shooter, 2000));

    JoystickButton rpmButton3 = new JoystickButton(m_controller, XboxController.Button.kB.value);
    rpmButton3.whileHeld(new RunShooterCommand(m_shooter, 2500));
    //: Climber control
    DPadTrigger ForwardClimbOutwardsButton = new DPadTrigger(m_controller, DPadTrigger.DPad.kUp);
    ForwardClimbOutwardsButton.whileActiveContinuous(new MoveClimberOutwards(m_frontClimber));

    DPadTrigger ForwardClimbInwardsButton = new DPadTrigger(m_controller, DPadTrigger.DPad.kRight);
    ForwardClimbInwardsButton.whileActiveContinuous(new MoveClimberInwards(m_frontClimber));

    DPadTrigger BackwardClimbOutwardsButtons = new DPadTrigger(m_controller, DPadTrigger.DPad.KLeft);
    BackwardClimbOutwardsButtons.whileActiveContinuous(new MoveClimberOutwards(m_backClimber));

    DPadTrigger BackwardClimbInwardButton = new DPadTrigger(m_controller, DPadTrigger.DPad.KDown);
    BackwardClimbInwardButton.whileActiveContinuous(new MoveClimberInwards(m_backClimber));
    //: Intake control
    JoystickButton deployIntakeButton = new JoystickButton(m_controller, XboxController.Button.kY.value);
    deployIntakeButton.whileHeld(new DeployIntakeCommand(m_intake));
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

//    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
//            new Pose2d(0, 0, new Rotation2d(0)),
//            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
//            new Pose2d(3, 0, new Rotation2d(0)),
//            Constants.kTrajectoryConfig
//    );
//
//    return new DriveAlongTrajectoryCommand(m_drive, trajectory)
//            .andThen(() -> m_drive.tankDriveVolts(0, 0));
  }
}
