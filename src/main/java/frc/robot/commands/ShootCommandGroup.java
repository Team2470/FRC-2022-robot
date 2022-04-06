// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.commands.RunConveyorCommand.Direction;
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
        new AutoAlign(vision, drive),
        new MoveConveyorDistanceCommand(conveyor, -Units.inchesToMeters(4)),
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
