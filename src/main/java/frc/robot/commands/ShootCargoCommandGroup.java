package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

public class ShootCargoCommandGroup extends SequentialCommandGroup {

    public ShootCargoCommandGroup(Shooter shooter, Conveyor conveyor, int endingCargoCount) {
        addCommands(
            new MoveConveyorDistanceCommand(conveyor,-1), // We could do this every time... would delay shooter but we don't need more senors/complicated logic
            new ParallelDeadlineGroup(
                // Command deadline
                new SequentialCommandGroup(
                    new WaitForShooterRPMCommand(shooter, 1500),
                    // We may want to move conveyor a fixed distance before checking the sensor
                    new MoveConveyorDistanceCommand(conveyor, 2), // This is the diameter of the ball is 9 inches
                    new RunConveyorCommand(conveyor, RunConveyorCommand.Direction.kUp)
                        // Interrupt the conveyor when there are no more balls
                        // This assumes that this sensors in the conveyor stops seeing the ball once it is shot
                        // Typically endingCargoCount will be 0 or 1, depends on if we started with 2 or 1 balls
                        .withInterrupt(() -> conveyor.capturedBallCount() == endingCargoCount)
                ),

                // Stuff to run parallel
                new RunShooterCommand(shooter, 1500)
            )
        );
    }

}
