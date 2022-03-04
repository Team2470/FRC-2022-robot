package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.Constants;
import frc.robot.subsystems.FrontClimber;

public class FrontClimbAngleCommandFactory {
  public static Command make(FrontClimber climber, Rotation2d setpoint) {
    return new SelectCommand(
        () -> {
          double error = Math.abs(climber.getAngle().minus(setpoint).getDegrees());
          SmartDashboard.putNumber("Arm Error", error);
          if (error > 0) {
            return new MoveFrontClimberInwards(climber).alongWith(new PrintCommand("Inward"));
          } else {
            return new MoveFrontClimberOutwards(climber).alongWith(new PrintCommand("Outward"));
          }
        }
    ).withInterrupt(() -> Math.abs(climber.getAngle().minus(setpoint).getDegrees()) <= Constants.kBackClimberErrorBound);
  }
}