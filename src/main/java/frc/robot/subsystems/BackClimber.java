// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BackClimber extends SubsystemBase implements Climber {

  private WPI_TalonFX m_backClimber;
  private CANCoder m_backCanCoder; 

  /** Creates a new Climber. */
  public BackClimber() {
    m_backClimber = new WPI_TalonFX(Constants.kBackClimberTalonId, Constants.kCanivoreName);
    m_backCanCoder = new CANCoder(Constants.kBackCanCoderId, Constants.kCanivoreName);

  

    m_backCanCoder.configFactoryDefault();
    m_backCanCoder.configMagnetOffset(Constants.kBackClimberCanCoderOffset);
    m_backCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_backCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);


    m_backClimber.configFactoryDefault();
    m_backClimber.configForwardSoftLimitEnable(true);
    m_backClimber.configReverseSoftLimitEnable(true);
    m_backClimber.configReverseSoftLimitThreshold(Constants.kBackClimberReverseLimit);
    m_backClimber.configForwardSoftLimitThreshold(Constants.kBackClimberForwardLimit);

    m_backClimber.configRemoteFeedbackFilter(m_backCanCoder, 0);
    m_backClimber.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Back Angle", m_backCanCoder.getPosition());
    SmartDashboard.putNumber("Climber Back Absolute Angle", m_backCanCoder.getAbsolutePosition());
    SmartDashboard.putNumber("Climber Back Selected Sensor position", m_backClimber.getSelectedSensorPosition());
  }

  public void startClimbMotor(int direction, double speed) {
    m_backClimber.set(ControlMode.PercentOutput, direction*speed);
  }

  // TODO: Adjust to match hardware
  public void startOutwardClimb() {
    m_backClimber.set(ControlMode.PercentOutput, Constants.kClimberSpeed);
  }

  public void startInwardClimb() {
    m_backClimber.set(ControlMode.PercentOutput, -Constants.kClimberSpeed);
  }

  public Rotation2d getAngle() { return Rotation2d.fromDegrees(this.m_backCanCoder.getAbsolutePosition()); }

  public void stop() {
    m_backClimber.neutralOutput();
  }
}
