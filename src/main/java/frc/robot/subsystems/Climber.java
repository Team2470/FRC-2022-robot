package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Climber extends Subsystem {
  void startOutwardClimb();

  void startInwardClimb();

  Rotation2d getAngle();

  void stop();
}
