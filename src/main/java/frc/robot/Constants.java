// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    public static final int kDriveTalonRightAId = 11;
    public static final int kDriveTalonRightBId = 12;
    public static final int kDriveTalonLeftAId  = 13;
    public static final int kDriveTalonLeftBId  = 14;

    // Controllers

    public static final int kControllerA = 0;

    // Drive

    public static final double ksVolts = 0.5805;
    public static final double kvVoltSecondsPerMeter = 5.9;
    public static final double kaVoltSecondsSquaredPerMeter = 0.63211;

    // kP value for PID
    public static final double kPDriveVel = 7.524;
}
