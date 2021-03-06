// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {

  // Drive
  public static final double gearRatio = 12.75; //2020 robot ratio: 26.04
  public static final double wheelDiameterMeters = Units.inchesToMeters(6);
  public static final double encoderCounts = 2048;

  public static double encoderCountsToMeters(double counts) {
    return counts * ((Math.PI * wheelDiameterMeters) / (encoderCounts * gearRatio));
  }

  public static double encoderRateToMetersPerSecond(double rate) {
    return encoderCountsToMeters(rate) * 10;
  }


  // kP value for PID
  public static final double kPDriveVel = 7.524;
  public static final double kTrackwidthMeters = Units.inchesToMeters(24);


  public static final DifferentialDriveKinematics kDriveKinematics =
      new DifferentialDriveKinematics(kTrackwidthMeters);

  public static final double kMaxSpeedMetersPerSecond = 1.09; //3 feet per second
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;

  // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;

  private final WPI_TalonFX m_leftLeader;
  private final WPI_TalonFX m_leftFollower;
  private final WPI_TalonFX m_rightLeader;
  private final WPI_TalonFX m_rightFollower;

  private final DifferentialDrive m_drive;

  // The gyro sensor
  //private final PigeonIMU m_gyro;

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /**
   * Creates a new Drive.
   */
  public Drive(WPI_TalonSRX gyroTalon) {
    m_leftLeader = new WPI_TalonFX(Constants.kDriveTalonLeftAId, Constants.kCanivoreName);
    m_leftFollower = new WPI_TalonFX(Constants.kDriveTalonLeftBId, Constants.kCanivoreName);
    m_leftFollower.follow(m_leftLeader);
    m_leftFollower.configFactoryDefault();
    m_leftFollower.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 1000);
    m_leftFollower.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1000);

    m_rightLeader = new WPI_TalonFX(Constants.kDriveTalonRightAId, Constants.kCanivoreName);
    m_rightFollower = new WPI_TalonFX(Constants.kDriveTalonRightBId, Constants.kCanivoreName);
    m_rightFollower.follow(m_rightLeader);
    m_rightFollower.configFactoryDefault();
    m_rightFollower.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 1000);
    m_rightFollower.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1000);

    m_leftLeader.setInverted(false);
    m_leftFollower.setInverted(false);
    m_rightLeader.setInverted(true);
    m_rightFollower.setInverted(true);

    // Use brake mode to stop our robot coasting.
    m_leftLeader.setNeutralMode(NeutralMode.Coast);
    m_leftFollower.setNeutralMode(NeutralMode.Coast);
    m_rightLeader.setNeutralMode(NeutralMode.Coast);
    m_rightFollower.setNeutralMode(NeutralMode.Coast);
    
    m_drive = new DifferentialDrive(m_leftLeader, m_rightLeader);
  //  m_gyro = new PigeonIMU(gyroTalon);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(getHeading());
  }

  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left, right, false);
  }

  public void arcadeDrive(double forward, double rotate) {
    m_drive.arcadeDrive(forward, rotate);
  }

  public void curvatureDrive(double xSpeed, double zRotation, boolean allowTurnInPlace) {
    xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
    zRotation = Math.copySign(zRotation * zRotation, zRotation);
    m_drive.curvatureDrive(xSpeed, zRotation, allowTurnInPlace);
  }


  public void stop() {
    m_drive.stopMotor();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        getHeading(), getLeftEncoderDistance(), getRightEncoderDistance());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderRate(), getRightEncoderRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, getHeading());
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftLeader.setVoltage(leftVolts);
    m_rightLeader.setVoltage(rightVolts);
    m_drive.feed();
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_leftLeader.setSelectedSensorPosition(0, 0, 0);
    m_rightLeader.setSelectedSensorPosition(0, 0, 0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder distance.
   *
   * @return meters
   */
  public double getLeftEncoderDistance() {
    return encoderCountsToMeters(m_leftLeader.getSelectedSensorPosition());
  }

  /**
   * Gets the right drive encoder distance.
   *
   * @return meters
   */
  public double getRightEncoderDistance() {
    return encoderCountsToMeters(m_rightLeader.getSelectedSensorPosition());
  }

  /**
   * Gets the left drive encoder rate.
   *
   * @return meters/second
   */
  public double getLeftEncoderRate() {
    return encoderRateToMetersPerSecond(m_leftLeader.getSelectedSensorVelocity());
  }

  /**
   * Gets the right drive encoder rate.
   *
   * @return meters/second
   */
  public double getRightEncoderRate() {
    return encoderRateToMetersPerSecond(m_rightLeader.getSelectedSensorVelocity());
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    //m_gyro.setFusedHeading(0);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getHeading() {
   // return Rotation2d.fromDegrees(m_gyro.getFusedHeading());
   return Rotation2d.fromDegrees(0);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeadingDegrees() {
    return getHeading().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    double[] xyz_dps = new double[3];
    //m_gyro.getRawGyro(xyz_dps);
    return xyz_dps[2];
  }
}
