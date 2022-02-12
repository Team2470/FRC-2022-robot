// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

import java.util.List;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // General

    public static final int kClockwise = 1;
    public static final int kCounterClockwise = -1;

    // CAN IDs

    public static final int kDriveTalonRightAId = 13;
    public static final int kDriveTalonRightBId = 14;
    public static final int kDriveTalonLeftAId  = 11;
    public static final int kDriveTalonLeftBId  = 12;
    public static final int kFrontClimberTalonId  = 10;
    public static final int kFrontCanCoderId = 9;
    public static final int kBackClimberTalonId = 8;
    public static final int kBackCanCoderId = 7;
    public static final String kCanivoreName = "Canivore0";
    public static final int kGyroTalon = 15;

    // Pneumatics

    public static final int kRatchetSolenoid = 0;

    public static final int kShooterNeoLeaderId = 1;
    public static final int kShooterNeoFollowerId= 3;
    // Controllers

    public static final int kControllerA = 0;


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

    
    // Drive

    public static final double ksVolts = 0.5805;
    public static final double kvVoltSecondsPerMeter = 5.9;
    public static final double kaVoltSecondsSquaredPerMeter = 0.63211;

    // kP value for PID
    public static final double kPDriveVel = 7.524;
  
    // Climber

    public static final int kFrontClimberReverseLimit  = -1024;
    public static final int kFrontClimberForwardLimit = 1024;
    public static final double kFrontClimberCanCoderOffset = 208;
    public static final double kClimberSpeed = 0.2;

    public static final int kBackClimberReverseLimit  = -1024;
    public static final int kBackClimberForwardLimit = 1024;
    public static final double kBackClimberCanCoderOffset = 208;


}
