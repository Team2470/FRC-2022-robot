// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.automation.ClimbCommandGroup;
import frc.robot.commands.automation.ShootCommandGroup;
import frc.robot.commands.climber.*;
import frc.robot.commands.conveyor.MoveConveyorDistanceCommand;
import frc.robot.commands.conveyor.RunConveyorCommand;
import frc.robot.commands.conveyor.RunConveyorCommand.Direction;
import frc.robot.commands.drive.AlignCommand;
import frc.robot.commands.drive.DriveDistanceCommand;
import frc.robot.commands.drive.DriveShooterWithSmartDashboardCommand;
import frc.robot.commands.drive.DriveWithGamepadCommand;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.RetractIntakeCommand;
import frc.robot.commands.shooter.RunShooterCommand;
import frc.robot.subsystems.*;

import java.util.Map;

import com.kennedyrobotics.auto.AutoSelector;
import com.kennedyrobotics.hardware.components.RevDigit;

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
  private final Joystick m_testpad = new Joystick(1);
  private final Joystick m_buttopad = new Joystick(2);

  //Auto
  private final RevDigit revDigit_;
  private final AutoSelector autoSelector_;



  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    DataLogManager.start();
    // Configure the button bindings
    configureButtonBindings();
    configureTestingCommands();

    m_drive.setDefaultCommand(new DriveWithGamepadCommand(m_drive, m_controller));
    m_intake.setDefaultCommand(new RetractIntakeCommand(m_intake));
    m_vision.init();
    revDigit_ = new RevDigit();
    revDigit_.display("BWMP");
    autoSelector_ = new AutoSelector(revDigit_, "1BLL",  new SequentialCommandGroup(
        new DriveDistanceCommand(m_drive, 3),
        new AlignCommand(m_vision, m_drive),
        new ShootCommandGroup(m_conveyor, m_shooter, m_vision, m_drive, 0)
    ));

    autoSelector_.registerCommand("2Ball", "2BLL",  new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new FunctionalCommand(
                () -> {
                },
                () -> {
                },
                (onEnd) -> {
                },
                m_conveyor::isSecondCargoDetected
                , m_conveyor
            ),
            new DeployIntakeCommand(m_intake),
            new DriveDistanceCommand(m_drive, 1)
        ),
        new AlignCommand(m_vision, m_drive),
        new ShootCommandGroup(m_conveyor, m_shooter, m_vision, m_drive, 0)
    ));
    // autoSelector_.registerCommand("bar", "BAR", new PrintCommand("Bar"));
    // autoSelector_.registerCommand("foobar2000", "FB2", new PrintCommand("Foobar200"));
    autoSelector_.initialize();

  }

  public void disabledInit() {
    m_vision.init();
  }

  public void teleopInit() {
    m_shooter.init();
    m_vision.init();
  }

  public void autoInit() {
    m_shooter.init();
    m_vision.init();
  }

  private void configureTestingCommands() {
    ShuffleboardLayout visionCommands = Shuffleboard.getTab("Commands")
        .getLayout("Vision", BuiltInLayouts.kList)
        .withSize(2, 2)
        .withPosition(0, 0)
        .withProperties(Map.of("Label position", "HIDDEN"));
    //visionCommands.add(new AlignCommand(m_vision, m_drive));
    visionCommands.add(new InstantCommand(() -> m_vision.setCameraMode(Vision.CameraMode.kCalibration), m_vision));
    //visionCommands.add(new InstantCommand(() -> m_vision.setCameraMode(Vision.CameraMode.kDriving), m_vision));


    ShuffleboardLayout conveyorCommands = Shuffleboard.getTab("Commands")
        .getLayout("Conveyor", BuiltInLayouts.kGrid)
        .withSize(2, 2)
        .withPosition(2, 0)
        .withProperties(Map.of("Label position", "HIDDEN"));
    conveyorCommands.add("Move 1 in. up", new MoveConveyorDistanceCommand(m_conveyor, Units.inchesToMeters(5)));
    conveyorCommands.add("Move 1 in. down", new MoveConveyorDistanceCommand(m_conveyor, Units.inchesToMeters(-5)));
    conveyorCommands.add("Move up", new RunConveyorCommand(m_conveyor, RunConveyorCommand.Direction.kUp));
    conveyorCommands.add("Move down", new RunConveyorCommand(m_conveyor, RunConveyorCommand.Direction.kDown));
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Conveyor Control
    //JoystickButton conveyorUp = new JoystickButton(m_buttopad, 8);
    //conveyorUp.whileHeld(new RunConveyorCommand(m_conveyor, Direction.kUp));

    JoystickButton conveyorDown = new JoystickButton(m_buttopad, 12);
    conveyorDown.whileHeld(new RunConveyorCommand(m_conveyor, RunConveyorCommand.Direction.kDown));
    // //: Shooter control
    JoystickButton rpmButton1 = new JoystickButton(m_buttopad, 9);
    rpmButton1.whileHeld(new RunShooterCommand(m_shooter, -800));

    JoystickButton rpmButton2 = new JoystickButton(m_buttopad, 10);
    rpmButton2.whileHeld(new RunShooterCommand(m_shooter, 800));

    JoystickButton rpmButton3 = new JoystickButton(m_buttopad, 11);
    rpmButton3.whileHeld(new RunShooterCommand(m_shooter, 2600));

    JoystickButton rpmControlButton = new JoystickButton(m_controller, XboxController.Button.kA.value);
    rpmControlButton.whileHeld(new DriveShooterWithSmartDashboardCommand(m_shooter));
    //: Climber control
    JoystickButton ForwardClimbOutwardsButton = new JoystickButton(m_buttopad, 2);
    ForwardClimbOutwardsButton.whileActiveContinuous(new MoveFrontClimberOutwards(m_frontClimber));

    JoystickButton ForwardClimbInwardsButton = new JoystickButton(m_buttopad, 1);
    ForwardClimbInwardsButton.whileActiveContinuous(new MoveFrontClimberInwards(m_frontClimber));

    JoystickButton BackwardClimbOutwardsButtons = new JoystickButton(m_buttopad, 4);
    BackwardClimbOutwardsButtons.whileActiveContinuous(new MoveBackClimberOutwards(m_backClimber));

    JoystickButton BackwardClimbInwardButton = new JoystickButton(m_buttopad, 3);
    BackwardClimbInwardButton.whileActiveContinuous(new MoveBackClimberInwards(m_backClimber));

    ClimbCommandGroup climbCommand = new ClimbCommandGroup(m_frontClimber, m_backClimber);

    JoystickButton NextStepButton = new JoystickButton(m_buttopad, 5);
    NextStepButton.whenPressed(climbCommand);

    JoystickButton BailButton = new JoystickButton(m_buttopad, 6);
    BailButton.cancelWhenPressed(climbCommand);

    JoystickButton ForceAdvanceButton = new JoystickButton(m_buttopad, 8);
    ForceAdvanceButton.cancelWhenPressed(climbCommand);
    ForceAdvanceButton.whenPressed(new InstantCommand(() -> climbCommand.advanceState()).andThen(()->climbCommand.schedule()));

    JoystickButton Reset = new JoystickButton(m_buttopad, 7);
    Reset.whenPressed(new PrintCommand("Resetting").andThen(ClimbCommandGroup::reset));

    JoystickButton Move30 = new JoystickButton(m_testpad, 5);
    Move30.whenPressed(new BackClimbAngleCommand(m_backClimber, Rotation2d.fromDegrees(30)).perpetually());

    JoystickButton Move90 = new JoystickButton(m_testpad, 6);
    Move90.whenPressed(new BackClimbAngleCommand(m_backClimber, Rotation2d.fromDegrees(90)).perpetually());

    JoystickButton Move125 = new JoystickButton(m_testpad, 7);
    Move125.whenPressed(new BackClimbAngleCommand(m_backClimber, Rotation2d.fromDegrees(125)).perpetually());

    JoystickButton ForwardClimbOutwardsButton2 = new JoystickButton(m_testpad, 2);
    ForwardClimbOutwardsButton2.whileActiveContinuous(new MoveFrontClimberOutwards(m_frontClimber));

    JoystickButton ForwardClimbInwardsButton2 = new JoystickButton(m_testpad, 1);
    ForwardClimbInwardsButton2.whileActiveContinuous(new MoveFrontClimberInwards(m_frontClimber));

    JoystickButton BackwardClimbOutwardsButtons2 = new JoystickButton(m_testpad, 4);
    BackwardClimbOutwardsButtons2.whileActiveContinuous(new MoveBackClimberOutwards(m_backClimber));

    JoystickButton BackwardClimbInwardButton2 = new JoystickButton(m_testpad, 3);
    BackwardClimbInwardButton2.whileActiveContinuous(new MoveBackClimberInwards(m_backClimber));

    //: Intake control
    JoystickButton deployIntakeButton = new JoystickButton(m_controller, XboxController.Button.kY.value);
    deployIntakeButton.whenHeld(
        new ParallelCommandGroup(
            new DeployIntakeCommand(m_intake),
            new RunConveyorCommand(m_conveyor, Direction.kUp)
        ).withInterrupt(m_conveyor::isFull)
    );

    JoystickButton shootButton = new JoystickButton(m_controller, XboxController.Button.kStart.value);
    shootButton.whileHeld(
        new SelectCommand(
            Map.of(
                0, new PrintCommand("No cargo. Refusing to shoot"),
                1, new ShootCommandGroup(m_conveyor, m_shooter, m_vision, m_drive, 0),
                2, new ShootCommandGroup(m_conveyor, m_shooter, m_vision, m_drive, 1)
            ), m_conveyor::capturedCargoCount)
    );

  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    /*
    //One Ball Auto 80 degree
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
    */

    
    //One Ball Auto 60 degree
    return new SequentialCommandGroup(
        new DriveDistanceCommand(m_drive, 3),
        new AlignCommand(m_vision, m_drive),
        new ShootCommandGroup(m_conveyor, m_shooter, m_vision, m_drive, 0)
    );
    

    /*
    //Two Ball Auto 60 degree
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new FunctionalCommand(
                () -> {
                },
                () -> {
                },
                (onEnd) -> {
                },
                m_conveyor::isSecondCargoDetected
                , m_conveyor
            ),
            new DeployIntakeCommand(m_intake),
            new DriveDistanceCommand(m_drive, 1)
        ),
        new AlignCommand(m_vision, m_drive),
        new ShootCommandGroup(m_conveyor, m_shooter, m_vision, 0)
    );*/
    
    //Two Ball Auto 80 degree
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
