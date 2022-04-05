// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Vision extends SubsystemBase {
  private final NetworkTable m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  private final NetworkTableEntry m_tx = m_limelightTable.getEntry("tx");
  private final NetworkTableEntry m_ty = m_limelightTable.getEntry("ty");
  private final NetworkTableEntry m_ta = m_limelightTable.getEntry("ta");
  private final NetworkTableEntry m_tv = m_limelightTable.getEntry("tv");
  private final NetworkTableEntry m_usbCam = m_limelightTable.getEntry("stream");
  private final MedianFilter m_distanceFilter = new MedianFilter(5);
  private double m_filteredDistance;

  public enum StreamMode {
    kSideBySide(0),
    kLimelightPrimary(1),
    kWebcamPrimary(2);
    public final int value;
    StreamMode(int value) { this.value = value; }
  }

  public enum LEDMode {
    kOff(1),
    kOn(3);
    public final int value;
    LEDMode(int value) { this.value = value; }
  }

  public enum ProcessingMode {
    kPipeline(0),
    kDriver(1);
    public final int value;
    ProcessingMode(int value) { this.value = value; }
  }

  public enum CameraMode {
    kCalibration(StreamMode.kLimelightPrimary, ProcessingMode.kPipeline),
    kDriving(StreamMode.kWebcamPrimary, ProcessingMode.kDriver),
    kShooting(StreamMode.kLimelightPrimary, ProcessingMode.kPipeline),
    kClimbing(StreamMode.kLimelightPrimary,  ProcessingMode.kDriver);
    public final StreamMode streamMode;
    public final ProcessingMode processingMode;
    CameraMode(StreamMode streamMode, ProcessingMode processingMode) {
      this.streamMode = streamMode;
      this.processingMode = processingMode;
    }
  }


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
    m_filteredDistance = m_distanceFilter.calculate(getTargetDistance());

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Distance to Target", getTargetDistance());
    SmartDashboard.putNumber("Filtered distnce", getFilteredDistance());
    SmartDashboard.putNumber("Desired RPM", getRPM());
  }

  public void setStreamMode(StreamMode mode) { m_limelightTable.getEntry("stream").setNumber(mode.value); }
  public void setProcessingMode(ProcessingMode mode) { m_limelightTable.getEntry("camMode").setNumber(mode.value); }
  public void setLEDMode(LEDMode mode) { m_limelightTable.getEntry("ledMode").setNumber(mode.value); }
  public void setCameraMode(CameraMode mode) {
    setStreamMode(mode.streamMode);
    setProcessingMode(mode.processingMode);
    switch(mode.processingMode) {
      case kDriver:
        setLEDMode(LEDMode.kOff);
        break;
      case kPipeline:
        setLEDMode(LEDMode.kOn);
        break;
    }
  }

  public void init() {
    setCameraMode(CameraMode.kDriving);
  }

  public int getRPM() {
    // Add offset from base of target to center of hoop
    double distance = getFilteredDistance() + 34;

    double omega = 10.485 * distance + 1694.7;

    int rpm = (int) Math.round(omega);

    return Math.min(Math.max(rpm, 1000), 3500);
  }

  public boolean isShotPossible() {
    return getRPM() < 3500 && getFilteredDistance() > 5 * 12;
  }

  public double getFilteredDistance() {
    return m_filteredDistance;
  }

  /**
   * Finds the distance in inches from the front of the robot (no bumper) to the base of the target
   *
   * @return Distance to target
   */
  public double getTargetDistance() {
    if (m_tv.getDouble(0.0) == 1.0) {
      // 104 = height of target
      // 45 = height of camera
      // 29 = distance from front of robot to camera
      return (104 - 45) / getVerticalAngle().getTan() - 29;
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
      return Rotation2d.fromDegrees(m_ty.getDouble(0.0)).plus(Rotation2d.fromDegrees(24));
    } else {
      return Rotation2d.fromDegrees(0.0);
    }
  }
}
