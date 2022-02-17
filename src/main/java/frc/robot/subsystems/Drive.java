// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
//import com.ctre.phoenix.motorcontrol.VictorSP;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {



  private VictorSP m_leftLeader;
 // private VictorSP m_leftFollower;
  private VictorSP m_rightLeader;
  //private VictorSP m_rightFollower;

  private DifferentialDrive m_drive;



  // Odometry class for tracking robot pose
  
  /** Creates a new Drive. */
  public Drive() {
    m_leftLeader = new VictorSP(9);
   
    

    m_rightLeader = new VictorSP(8);
   

    m_leftLeader.setInverted(false);
    
    m_rightLeader.setInverted(true);
    

    // Use brake mode to stop our robot coasting.
   
    
    m_drive = new DifferentialDrive(m_leftLeader, m_rightLeader);

    
    
  }




  public void arcadeDrive(double forward, double rotate){
    m_drive.arcadeDrive(forward, rotate);
  }

  public void curvatureDrive(double xSpeed, double zRotation, boolean allowTurnInPlace){
    xSpeed = Math.copySign(xSpeed*xSpeed, xSpeed)*0.5;
    zRotation = Math.copySign(zRotation*zRotation, zRotation);
    m_drive.curvatureDrive(xSpeed, zRotation, allowTurnInPlace);
  }


  public void stop(){
    m_drive.stopMotor();
  }
  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    
  }

  
}
