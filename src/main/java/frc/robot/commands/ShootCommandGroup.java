// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootCommandGroup extends SequentialCommandGroup {
  /** Creates a new ShootCommandGroup. */
  public ShootCommandGroup(Conveyor conveyor, Shooter shooter, Vision vision, int endingCargoCount) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveConveyorDistanceCommand(conveyor, -1),
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new WaitForShooterRPMCommand(shooter, vision::getRPM),
          new MoveConveyorDistanceCommand(conveyor, 2),
          new RunConveyorCommand(conveyor, RunConveyorCommand.Direction.kUp)
            .withInterrupt(() -> conveyor.cargoCount()==endingCargoCount)
        ),
        new RunShooterCommand(shooter, vision::getRPM)
      )
    );
  }
}
