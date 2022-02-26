// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kennedyrobotics.triggers.DPadTrigger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.commands.*;
import frc.robot.commands.RunConveyorCommand.Direction;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Vision;

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
  private final Vision m_vision = new Vision();



  // Controller
  private final XboxController m_controller = new XboxController(Constants.kControllerA);
  private final Joystick m_buttopad= new Joystick(1);
 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureTestingCommands();

    m_drive.setDefaultCommand(new DriveWithGamepadCommand(m_drive,m_controller));
    m_intake.setDefaultCommand(new RetractIntakeCommand(m_intake));
  }

  private void configureTestingCommands() {
    ShuffleboardLayout visionCommands = Shuffleboard.getTab("Commands")
    .getLayout("Vision", BuiltInLayouts.kList)
    .withSize(2,2)
    .withPosition(8,0)
    .withProperties(Map.of("Label position", "HIDDEN"));
    // visionCommands.add(new NamedInstantCommand("Driver Mode", () -> m_vision.setDriverMode(true), m_vision));
    // visionCommands.add(new NamedInstantCommand("Vision Mode", () -> m_vision.setDriverMode(false), m_vision));
    // visionCommands.add(new NamedInstantCommand("Conveyor View", () -> m_vision.viewConveyor(true), m_vision));
    // visionCommands.add(new NamedInstantCommand("Target View", () -> m_vision.viewConveyor(false), m_vision));
    visionCommands.add(new AutoAlign(m_vision, m_drive));
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Conveyor Control
    JoystickButton conveyorUp = new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);
    conveyorUp.whileHeld(new RunConveyorCommand(m_conveyor, Direction.kUp));

    JoystickButton conveyorDown = new JoystickButton(m_controller, XboxController.Button.kRightBumper.value);
    conveyorDown.whileHeld(new RunConveyorCommand(m_conveyor, Direction.kDown));
    //: Shooter control
    JoystickButton rpmButton1 = new JoystickButton(m_buttopad, 8);
    rpmButton1.whileHeld(new RunShooterCommand(m_shooter, () -> 2800));

    JoystickButton rpmButton2 = new JoystickButton(m_buttopad, 7);
    rpmButton2.whileHeld(new RunShooterCommand(m_shooter, () -> 2600));

    JoystickButton rpmButton3 = new JoystickButton(m_buttopad, 6);
    rpmButton3.whileHeld(new RunShooterCommand(m_shooter, () -> 2500));
    //: Climber control
    JoystickButton ForwardClimbOutwardsButton = new JoystickButton(m_buttopad, 2);
    ForwardClimbOutwardsButton.whileActiveContinuous(new MoveFrontClimberOutwards(m_frontClimber));

    JoystickButton ForwardClimbInwardsButton = new JoystickButton(m_buttopad, 1);
    ForwardClimbInwardsButton.whileActiveContinuous(new MoveFrontClimberInwards(m_frontClimber));

    JoystickButton BackwardClimbOutwardsButtons = new JoystickButton(m_buttopad, 4);
    BackwardClimbOutwardsButtons.whileActiveContinuous(new MoveBackClimberOutwards(m_backClimber));

    JoystickButton BackwardClimbInwardButton = new JoystickButton(m_buttopad, 3);
    BackwardClimbInwardButton.whileActiveContinuous(new MoveBackClimberInwards(m_backClimber));

    //JoystickButton MoveClimbToAngle = new JoystickButton(m_buttopad, 5);
    //MoveClimbToAngle.whenHeld(new ClimbAngleCommand(m_backClimber, Rotation2d.fromDegrees(10)));

    //: Intake control
    JoystickButton deployIntakeButton = new JoystickButton(m_controller, XboxController.Button.kY.value);
    // deployIntakeButton.whileHeld(new DeployIntakeCommand(m_intake));
    deployIntakeButton.whenHeld(
      new ParallelCommandGroup(
        new DeployIntakeCommand(m_intake),
        new RunConveyorCommand(m_conveyor, Direction.kUp)
      ).withInterrupt(m_conveyor::isFull)
    );
    
    JoystickButton shootButton = new JoystickButton(m_controller, XboxController.Button.kStart.value);
    shootButton.whenPressed(
      new SelectCommand(
        Map.of(
          0, new PrintCommand("No cargo. Refusing to shoot"),
          1, new ShootCommandGroup(m_conveyor, m_shooter, m_vision, 0),
          2, new ShootCommandGroup(m_conveyor, m_shooter, m_vision, 1)
        ), m_conveyor::capturedCargoCount)
      );

      JoystickButton conveyorDownButton = new JoystickButton(m_buttopad, 5);
      conveyorDownButton.whenHeld(new RunConveyorCommand(m_conveyor,Direction.kDown));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Return null for no autonomous command
    return new SequentialCommandGroup(
      new AutoDrive(m_drive).withTimeout(1.5),
      new ParallelRaceGroup(
           new RunShooterCommand(m_shooter, 2500),
           new SequentialCommandGroup(

               new WaitCommand(2),
               new RunConveyorCommand(m_conveyor, RunConveyorCommand.Direction.kUp).withTimeout(5)
       ))

      
            // new RunShooterCommand(m_shooter, 2500),
            // new SequentialCommandGroup(
            //         new WaitCommand(2),
            //         new RunConveyorCommand(m_conveyor, RunConveyorCommand.Direction.kUp).withTimeout(5)
            // )
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
