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

    // Pneumatics

    public static final int kRatchetSolenoid = 0;

    public static final int kShooterNeoLeaderId = 1;
    public static final int kShooterNeoFollowerId= 3;
    // Controllers

    public static final int kControllerA = 0;
  
    // Climber

    public static final int kFrontClimberReverseLimit  = -1024;
    public static final int kFrontClimberForwardLimit = 1024;
    public static final double kFrontClimberCanCoderOffset = 208;
    public static final double kClimberSpeed = 0.2;

    public static final int kBackClimberReverseLimit  = -1024;
    public static final int kBackClimberForwardLimit = 1024;
    public static final double kBackClimberCanCoderOffset = 208;


}
