// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootCommandGroup extends SequentialCommandGroup {
  private final Vision m_vision;

  /**
   * Creates a new ShootCommandGroup.
   */
  public ShootCommandGroup(Conveyor conveyor, Shooter shooter, Vision vision, int endingCargoCount) {
    m_vision = vision;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new MoveConveyorDistanceCommand(conveyor, -1),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new WaitForShooterRPMCommand(shooter, this::getRPM),
                new MoveConveyorDistanceCommand(conveyor, 2),
                new RunConveyorCommand(conveyor, RunConveyorCommand.Direction.kUp)
                    .withInterrupt(() -> conveyor.capturedCargoCount() == endingCargoCount)
            ),
            new RunShooterCommand(shooter, this::getRPM)
        )
    );
  }

  private int getRPM() {
    Rotation2d angle = m_vision.getVerticalAngle();
    double distance = m_vision.geTargetDistanceM();

    double v0 = (-9.81 * distance * distance) / (2 * distance * angle.getSin() - Constants.kHubHeightM);
    double omega = (60 * v0) / (2 * Math.PI * Constants.kWheelRadiusM);

    return (int) Math.round(omega);
  }
}
