// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {

  private WPI_TalonFX m_leftLeader;
  private WPI_TalonFX m_leftFollower;
  private WPI_TalonFX m_rightLeader;
  private WPI_TalonFX m_rightFollower;

  private DifferentialDrive m_drive;
  /** Creates a new Drive. */
  public Drive() {
    m_leftLeader = new WPI_TalonFX(Constants.kDriveTalonLeftAId);
    m_leftFollower = new WPI_TalonFX(Constants.kDriveTalonLeftBId);
    m_leftFollower.follow(m_leftLeader);

    m_rightLeader = new WPI_TalonFX(Constants.kDriveTalonRightAId);
    m_rightFollower = new WPI_TalonFX(Constants.kDriveTalonRightBId);
    m_rightFollower.follow(m_rightLeader);

    m_leftLeader.setInverted(false);
    m_leftFollower.setInverted(false);
    m_rightLeader.setInverted(true);
    m_rightFollower.setInverted(true);

    // Use brake mode to stop our robot coasting.
    m_leftLeader.setNeutralMode(NeutralMode.Brake);
    m_leftFollower.setNeutralMode(NeutralMode.Brake);
    m_rightLeader.setNeutralMode(NeutralMode.Brake);
    m_rightFollower.setNeutralMode(NeutralMode.Brake);
    
    m_drive = new DifferentialDrive(m_leftLeader, m_rightLeader);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
}
