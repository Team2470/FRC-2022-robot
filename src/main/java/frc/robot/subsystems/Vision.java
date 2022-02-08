// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;



public class Vision extends SubsystemBase {
private NetworkTable m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
private NetworkTableEntry m_tx = m_limelightTable.getEntry("tx");
private NetworkTableEntry m_ty = m_limelightTable.getEntry("ty");
private NetworkTableEntry m_ta = m_limelightTable.getEntry("ta");

//read values periodically
double x = m_tx.getDouble(0.0);
double y = m_ty.getDouble(0.0);
double area = m_ta.getDouble(0.0);

//post to smart dashboard periodically
SmartDashboard.putNumber("LimelightX", x);
SmartDashboard.putNumber("LimelightY", y);
SmartDashboard.putNumber("LimelightArea", area);
  
  /** Creates a new Vision. */
  public Vision() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
