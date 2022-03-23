// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Vision extends SubsystemBase {
  private final NetworkTable m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  private final NetworkTableEntry m_tx = m_limelightTable.getEntry("tx");
  private final NetworkTableEntry m_ty = m_limelightTable.getEntry("ty");
  private final NetworkTableEntry m_ta = m_limelightTable.getEntry("ta");
  private final NetworkTableEntry m_tv = m_limelightTable.getEntry("tv");
  private final NetworkTableEntry m_usbCam = m_limelightTable.getEntry("stream");


  /**
   * Creates a new Vision.
   */
  public Vision() {
    m_usbCam.setNumber(1);
  }

  @Override
  public void periodic() {
    //read values periodically
    double x = m_tx.getDouble(0.0);
    double y = m_ty.getDouble(0.0);
    double area = m_ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Distance to Target", getTargetDistance());
  }

  /**
   * Finds the distance in meters from the front of the robot (no bumper) to the base of the target
   *
   * @return Distance to target
   */
  public double getTargetDistance() {
    if (m_tv.getDouble(0.0) == 1.0) {
      // 29 = distance from front of robot to camera
      return (Constants.kTargetHeightM - Constants.kCameraHeightM) / getVerticalAngle().getTan() - 29;
    } else {
      return 0;
    }
  }

  /**
   * Returns if target is found
   *
   * @return true if found
   */
  public boolean getTargetFound() {
    return m_tv.getDouble(0.0) == 1.0;
  }

  public Rotation2d getHorizontalAngle() {
    if (m_tv.getDouble(0.0) == 1.0) {
      return Rotation2d.fromDegrees(m_tx.getDouble(0.0));
    } else {
      return Rotation2d.fromDegrees(0);
    }
  }

  public Rotation2d getVerticalAngle() {
    if (m_tv.getDouble(0.0) == 1.0) {
      return Rotation2d.fromDegrees(m_ty.getDouble(0.0)).plus(Constants.kCameraAngle);
    } else {
      return Rotation2d.fromDegrees(0.0);
    }
  }
}
