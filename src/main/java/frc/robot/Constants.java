// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Physical
  public static final double kHubHeightM = 1.6;
  public static final double kWheelRadiusM = Units.inchesToMeters(3);
  public static final double kConveyorWheelRadiusM = Units.inchesToMeters(1.416);

  // General

  public static final int kClockwise = 1;
  public static final int kCounterClockwise = -1;

  // CAN IDs

  // CANivore bus
  public static final String kCanivoreName = "";
  public static final int kDriveTalonRightAId = 13;
  public static final int kDriveTalonRightBId = 14;
  public static final int kDriveTalonLeftAId = 11;
  public static final int kDriveTalonLeftBId = 12;
  public static final int kFrontClimberTalonId = 10;
  public static final int kFrontClimberFollowerTalonId = 9;
  public static final int kBackClimberTalonId = 8;

  public static final int kFrontCanCoderId = 9;
  public static final int kBackCanCoderId = 7;


  // rio bus

  public static final int kConveyorMotorID = 18;
  public static final int kIntakeMotorId = 5;
  public static final int kConveyorCanCoder = 6;


  // Conveyor constants

  public static final int kConveyorUp = -1;
  public static final int kConveyorDown = 1;
  public static final double kConveyorSpeed = 0.5;
  public static final double kConveyorSpeedDown = -0.75;
  public static final double kConveyorErrorBoundM = Units.inchesToMeters(0.1);

  //DIO
  public static final int kConveyorFirstCargoChannel = 8;
  public static final int kConveyorSecondCargoChannel = 9;

  // Pneumatics

  public static final int kRatchetSolenoid = 0;
  public static final int kIntakeSolenoid = 1;

  public static final int kShooterNeoLeaderId = 1;
  public static final int kShooterNeoFollowerId = 3;
  // Controllers

  public static final int kControllerA = 0;

  //Vision constants
  public static final double kTargetHeightM = 2.52;
  public static final double kCameraHeightM = 1.143;
  public static final Rotation2d kCameraAngle = Rotation2d.fromDegrees(10);


  //Characterization Constraints
  public static final double ksVolts = 0.5805;
  public static final double kvVoltSecondsPerMeter = 5.9;
  public static final double kaVoltSecondsSquaredPerMeter = 0.63211;
  public static final double kPDriveVel = 7.524;
  public static final double kTrackwidthMeters = 0.6096; //24 inches
  public static final double kMaxSpeedMetersPerSecond = 1.09; //3 feet per second
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;

  public static final DifferentialDriveKinematics kDriveKinematics =
      new DifferentialDriveKinematics(kTrackwidthMeters);

  // Create a voltage constraint to ensure we don't accelerate too fast
  public static final DifferentialDriveVoltageConstraint kAutoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(
              Constants.ksVolts,
              Constants.kvVoltSecondsPerMeter,
              Constants.kaVoltSecondsSquaredPerMeter),
          Constants.kDriveKinematics,
          10);

  // Create config for trajectory
  public static final TrajectoryConfig kTrajectoryConfig =
      new TrajectoryConfig(
          Constants.kMaxSpeedMetersPerSecond,
          Constants.kMaxAccelerationMetersPerSecondSquared)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(Constants.kDriveKinematics)
          // Apply the voltage constraint
          .addConstraint(kAutoVoltageConstraint);
    // Climber
    public static final int kFrontClimberReverseLimit  = 100;
    public static final int kFrontClimberForwardLimit = 1120;
    public static final double kFrontClimberCanCoderOffset = 172;
    public static final double kFrontClimberSpeed = 0.4;
    
    public static final int kBackClimberReverseLimit  = 100;
    public static final int kBackClimberForwardLimit = 1390;
    public static final double kBackClimberCanCoderOffset = -104.35546875-91;
    public static final double kBackClimberSpeed = 0.2;



  public static final double kClimberErrorBound = 1; // Degrees

  // Intake
  public static final double kIntakeSpeed = 1.0;
}

