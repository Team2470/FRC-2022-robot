package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;


public class DriveAlongTrajectoryCommand extends RamseteCommand {
  private final Drive m_drive;
  private final Trajectory m_trajectory;

  public DriveAlongTrajectoryCommand(Drive drive, Trajectory trajectory) {
    super(
        trajectory,
        drive::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(
            Constants.ksVolts,
            Constants.kvVoltSecondsPerMeter,
            Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        drive::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        drive::tankDriveVolts,
        drive
    );
    m_drive = drive;
    // Stop when done
    //this.andThen(() -> m_drive.tankDriveVolts(0, 0));

    m_trajectory = trajectory;
    addRequirements(this.m_drive);
  }

  @Override
  public void initialize() {
    // Ramsete init
    super.initialize();
    // Reset our odometry before start
    m_drive.resetOdometry(m_trajectory.getInitialPose());
  }
}
