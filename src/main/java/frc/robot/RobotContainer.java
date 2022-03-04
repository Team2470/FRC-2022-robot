// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.RunConveyorCommand.Direction;
import frc.robot.subsystems.*;

import java.util.Map;

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
  private final Joystick m_buttopad = new Joystick(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureTestingCommands();

    m_drive.setDefaultCommand(new DriveWithGamepadCommand(m_drive, m_controller));
    m_intake.setDefaultCommand(new RetractIntakeCommand(m_intake));
  }

  private void configureTestingCommands() {
    ShuffleboardLayout visionCommands = Shuffleboard.getTab("Commands")
        .getLayout("Vision", BuiltInLayouts.kList)
        .withSize(2, 2)
        .withPosition(0, 0)
        .withProperties(Map.of("Label position", "HIDDEN"));
    visionCommands.add(new AutoAlign(m_vision, m_drive));
    ShuffleboardLayout conveyorCommands = Shuffleboard.getTab("Commands")
        .getLayout("Conveyor", BuiltInLayouts.kGrid)
        .withSize(2, 2)
        .withPosition(2, 0)
        .withProperties(Map.of("Label position", "HIDDEN"));
    conveyorCommands.add("Move 1 in. up", new MoveConveyorDistanceCommand(m_conveyor, Units.inchesToMeters(5)));
    conveyorCommands.add("Move 1 in. down", new MoveConveyorDistanceCommand(m_conveyor, Units.inchesToMeters(-5)));
    conveyorCommands.add("Move up", new RunConveyorCommand(m_conveyor, Direction.kUp));
    conveyorCommands.add("Move down", new RunConveyorCommand(m_conveyor, Direction.kDown));
    ShuffleboardLayout shooterCommands = Shuffleboard.getTab("Commands")
        .getLayout("Shooter", BuiltInLayouts.kGrid)
        .withSize(2, 2)
        .withPosition(0, 2);
    SendableChooser<Integer> rpmChooser = new SendableChooser<>();
    // Add options 2500 to 5000 with steps of 100
    rpmChooser.addOption("0", 0);
    for (int i = 2500; i <= 5000; i += 100) {
      rpmChooser.addOption(String.format("%s", i), i);
    }
    rpmChooser.setDefaultOption("0", 0);
    shooterCommands.add(rpmChooser);
    shooterCommands.add("Set RPM", new RunShooterCommand(m_shooter, rpmChooser::getSelected));
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Conveyor Control
    JoystickButton conveyorUp = new JoystickButton(m_buttopad, 8);
    conveyorUp.whileHeld(new RunConveyorCommand(m_conveyor, Direction.kUp));

    JoystickButton conveyorDown = new JoystickButton(m_buttopad, 12);
    conveyorDown.whileHeld(new RunConveyorCommand(m_conveyor, Direction.kDown));
    // //: Shooter control
    JoystickButton rpmButton1 = new JoystickButton(m_buttopad, 9);
    rpmButton1.whileHeld(new RunShooterCommand(m_shooter, Constants.kRPM1));

    JoystickButton rpmButton2 = new JoystickButton(m_buttopad, 10);
    rpmButton2.whileHeld(new RunShooterCommand(m_shooter, Constants.kRPM2));

    JoystickButton rpmButton3 = new JoystickButton(m_buttopad, 11);
    rpmButton3.whileHeld(new RunShooterCommand(m_shooter, Constants.kRPM3));
    //: Climber control
    if (true) {
      JoystickButton ForwardClimbOutwardsButton = new JoystickButton(m_buttopad, 2);
      ForwardClimbOutwardsButton.whileActiveContinuous(new MoveFrontClimberOutwards(m_frontClimber));

      JoystickButton ForwardClimbInwardsButton = new JoystickButton(m_buttopad, 1);
      ForwardClimbInwardsButton.whileActiveContinuous(new MoveFrontClimberInwards(m_frontClimber));

      JoystickButton BackwardClimbOutwardsButtons = new JoystickButton(m_buttopad, 4);
      BackwardClimbOutwardsButtons.whileActiveContinuous(new MoveBackClimberOutwards(m_backClimber));

      JoystickButton BackwardClimbInwardButton = new JoystickButton(m_buttopad, 3);
      BackwardClimbInwardButton.whileActiveContinuous(new MoveBackClimberInwards(m_backClimber));
    } else {
      // NEW COMMANDS
      JoystickButton MoveFrontClimbToAngle = new JoystickButton(m_buttopad, 1);
      MoveFrontClimbToAngle.whenPressed(new MoveFrontClimberCommand(
          m_frontClimber,
          Rotation2d.fromDegrees(Constants.kFrontAngle1)
      ));
      JoystickButton MoveFrontClimbToAngle2 = new JoystickButton(m_buttopad, 2);
      MoveFrontClimbToAngle2.whenPressed(new MoveFrontClimberCommand(
          m_frontClimber,
          Rotation2d.fromDegrees(Constants.kFrontAngle2)
      ));
      JoystickButton MoveFrontClimbToAngle3 = new JoystickButton(m_buttopad, 3);
      MoveFrontClimbToAngle3.whenPressed(new MoveFrontClimberCommand(
          m_frontClimber,
          Rotation2d.fromDegrees(Constants.kFrontAngle3)
      ));
      JoystickButton MoveFrontClimbToAngle4 = new JoystickButton(m_buttopad, 4);
      MoveFrontClimbToAngle4.whenPressed(new MoveFrontClimberCommand(
          m_frontClimber,
          Rotation2d.fromDegrees(Constants.kFrontAngle4)
      ));
    }

    JoystickButton MoveBackClimbToAngle = new JoystickButton(m_buttopad, 5);
    MoveBackClimbToAngle.whenPressed(new BackClimbAngleCommand(
        m_backClimber,
        Rotation2d.fromDegrees(Constants.kBackAngle1)
    ).perpetually());

    JoystickButton MoveBackClimbToAngle2 = new JoystickButton(m_buttopad, 6);
    MoveBackClimbToAngle2.whenPressed(new BackClimbAngleCommand(
        m_backClimber,
        Rotation2d.fromDegrees(Constants.kBackAngle2)
    ).perpetually());

    JoystickButton MoveBackClimbToAngle3 = new JoystickButton(m_buttopad, 7);
    MoveBackClimbToAngle3.whenPressed(new BackClimbAngleCommand(
        m_backClimber,
        Rotation2d.fromDegrees(Constants.kBackAngle3)
    ).perpetually());

    //: Intake control
    JoystickButton deployIntakeButton = new JoystickButton(m_controller, XboxController.Button.kY.value);
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

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
        new ParallelRaceGroup(
            new RunShooterCommand(m_shooter, 2500),
            new SequentialCommandGroup(
                new WaitCommand(2),
                new RunConveyorCommand(m_conveyor, RunConveyorCommand.Direction.kUp).withTimeout(5)
            )
        ),
        new DriveDistanceCommand(m_drive, 3)
    );
    /*return new SequentialCommandGroup(
        // Shoot ball 1
        new ParallelRaceGroup(
            new RunShooterCommand(m_shooter, 2500),
            new SequentialCommandGroup(
                new WaitCommand(2),
                new RunConveyorCommand(m_conveyor, RunConveyorCommand.Direction.kUp).withTimeout(5)
            )
        ),
        // Intake ball 2
        new ParallelDeadlineGroup(
            new FunctionalCommand(
                () -> {
                },
                () -> {
                },
                (onEnd) -> {
                },
                m_conveyor::isFirstCargoDetected
                , m_conveyor
            ),
            new DeployIntakeCommand(m_intake),
            new DriveDistanceCommand(m_drive, 1)
        ),
        // Shoot ball 2
        new DriveDistanceCommand(m_drive, -1),
        new ParallelRaceGroup(
            new RunShooterCommand(m_shooter, 2500),
            new SequentialCommandGroup(
                new WaitCommand(2),
                new RunConveyorCommand(m_conveyor, RunConveyorCommand.Direction.kUp).withTimeout(5)
            )
        )
    );*/
  }
}
