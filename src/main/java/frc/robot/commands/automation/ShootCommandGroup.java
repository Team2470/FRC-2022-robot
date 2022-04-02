// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automation;

import java.util.Map;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.conveyor.MoveConveyorDistanceCommand;
import frc.robot.commands.conveyor.RunConveyorCommand;
import frc.robot.commands.conveyor.RunConveyorCommand.Direction;
import frc.robot.commands.drive.AlignCommand;
import frc.robot.commands.shooter.RunShooterCommand;
import frc.robot.commands.shooter.WaitForShooterRPMCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootCommandGroup extends SequentialCommandGroup {
  /**
   * Creates a new ShootCommandGroup.
   */
  public ShootCommandGroup(Conveyor conveyor, Shooter shooter, Vision vision, Drive drive, int endingCargoCount) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> vision.setCameraMode(Vision.CameraMode.kShooting), vision),
        new InstantCommand(() -> shooter.setStateSpaceControlEnabled(true), shooter),
        new WaitUntilCommand(vision::getTargetFound),
        new WaitUntilCommand(vision::isShotPossible),
        new AlignCommand(vision, drive),
        new MoveConveyorDistanceCommand(conveyor, -Units.inchesToMeters(3)),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new WaitForShooterRPMCommand(shooter, vision::getRPM, 100),
                new RunConveyorCommand(conveyor, Direction.kUp)
                .withInterrupt(()-> shooter.getError()>5)
                //new MoveConveyorDistanceCommand(conveyor, Units.inchesToMeters(11)),
                //new RunConveyorCommand(conveyor, RunConveyorCommand.Direction.kUp)
                    //.withInterrupt(() -> conveyor.capturedCargoCount() == endingCargoCount)
                
            ),
            new RunShooterCommand(shooter, vision::getRPM)
        ),
        new SelectCommand(
          Map.of(
            0, new InstantCommand(() -> {}),
            1, new RunConveyorCommand(conveyor, Direction.kUp)
                  .withInterrupt(()-> conveyor.capturedCargoCount() == 1)
          ), () -> endingCargoCount
        ),
        new InstantCommand(() -> vision.setCameraMode(Vision.CameraMode.kDriving), vision),
        new InstantCommand(() -> shooter.setStateSpaceControlEnabled(false), shooter)
    );
  }
}
