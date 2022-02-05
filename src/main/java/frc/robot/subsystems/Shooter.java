// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private final static double kP = 0.0;
  private final static double kI = 0.0;
  private final static double kD = 0.0;
  private final static double kIz = 0.0;
  private final static double kFF = 0.0;
  private final static double kMinOutput = -1;
  private final static double kMaxOutput = 1;

  private final CANSparkMax m_shooterLeader;
  private final CANSparkMax m_shooterFollower;
  private final SparkMaxPIDController m_pidController;
  private final RelativeEncoder m_encoder;

  /** Creates a new Shooter. */
  public Shooter() {
    m_shooterLeader = new CANSparkMax(Constants.kShooterNeoLeaderId, MotorType.kBrushless);
    m_shooterFollower = new CANSparkMax(Constants.kShooterNeoFollowerId, MotorType.kBrushless);
    m_shooterFollower.follow(m_shooterLeader,true);

    m_shooterLeader.setSmartCurrentLimit(40);
    m_shooterFollower.setSmartCurrentLimit(40);
    m_shooterLeader.enableVoltageCompensation(10);
    m_shooterFollower.enableVoltageCompensation(10);

    m_pidController = m_shooterLeader.getPIDController();
    m_encoder = m_shooterLeader.getEncoder();

    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter velocity rpm", m_encoder.getVelocity());
  }
  public void setRPM(double setPoint) {
    m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    
    SmartDashboard.putNumber("Shooter setpoint rpm", setPoint);
  }
  public void setPercent(double percent) {
    m_shooterLeader.set(percent);
  }
  public void stop() {
    m_shooterLeader.stopMotor();
  }
}