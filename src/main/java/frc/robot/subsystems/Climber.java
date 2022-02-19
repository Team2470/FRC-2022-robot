package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Climber extends Subsystem {
    void startClimbMotor(int direction, double speed);

    Rotation2d getAngle();

    void stopClimbMotor();
}
