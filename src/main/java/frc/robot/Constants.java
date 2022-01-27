// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // CAN IDs

    public static final int kDriveTalonRightAId = 13;
    public static final int kDriveTalonRightBId = 14;
    public static final int kDriveTalonLeftAId  = 11;
    public static final int kDriveTalonLeftBId  = 12;
    public static final int kGyroTalon = 15;

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

    
}
