// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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

        // //Vision Values
        // double targetOffsetAngle_Vertical = m_ty.getDouble(0.0);
        // // how many degrees back is your limelight rotated from perfectly vertical?
        // double limelightMountAngleDegrees = 10;

        // // distance from the center of the Limelight lens to the floor
        // double limelightLensHeightInches = 44.5;

        // // distance from the target to the floor
        // double goalHeightInches = 102.0;

        // double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        // double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        // //calculate distance
        // double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);


        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("Distance to Target", geTargetDistanceM());
       // SmartDashboard.putNumber("Offset", targetOffsetAngle_Vertical);
       // SmartDashboard.putBoolean("Target Found", );
        // This method will be called once per scheduler run
    }

    /**
   * Finds the distance from the base of the robot to the base of the target
   * @return Distance in degrees from the base of the camera to the base of the target
   */
  public double geTargetDistanceM() {
    if(m_tv.getDouble(0.0) == 1.0)  {
      double yDegree = m_ty.getDouble(0.0);
      return (Constants.kTargetHeightM - Constants.kCameraHeightM) / Math.tan( (Constants.kCameraAngleD + yDegree) * (Math.PI / 180) );    
    } else {
      return 0;
    }

  }
  /**
   * Returns if target is found
   * @return true if found 
   */
  public boolean getTargetFound() {

    if(m_tv.getDouble(0.0) == 1.0) {

      return true;
    } else {
      return false;
    }
  }
  public double getHorizontalAngleD() {
    if(m_tv.getDouble(0.0) == 1.0)  {
      return m_tx.getDouble(0.0);  
    } else {
      return 0;
    }
}
}
