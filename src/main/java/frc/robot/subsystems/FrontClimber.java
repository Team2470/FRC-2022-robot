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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FrontClimber extends SubsystemBase {

  private WPI_TalonFX m_frontClimber;
  private CANCoder m_frontCanCoder; 

  /** Creates a new Climber. */
  public FrontClimber() {
    m_frontClimber = new WPI_TalonFX(Constants.kFrontClimberTalonId, Constants.kCanivoreName);
    m_frontCanCoder = new CANCoder(Constants.kFrontCanCoderId, Constants.kCanivoreName);

  

    m_frontCanCoder.configFactoryDefault();
    m_frontCanCoder.configMagnetOffset(Constants.kFrontClimberCanCoderOffset);
    m_frontCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_frontCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);


    m_frontClimber.configFactoryDefault();
    m_frontClimber.configForwardSoftLimitEnable(true);
    m_frontClimber.configReverseSoftLimitEnable(true);
    m_frontClimber.configReverseSoftLimitThreshold(Constants.kFrontClimberReverseLimit);
    m_frontClimber.configForwardSoftLimitThreshold(Constants.kFrontClimberForwardLimit);

    m_frontClimber.configRemoteFeedbackFilter(m_frontCanCoder, 0);
    m_frontClimber.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("CLimber Front Angle", m_frontCanCoder.getPosition());
    SmartDashboard.putNumber("CLimber Front Absolute Angle", m_frontCanCoder.getAbsolutePosition());
    SmartDashboard.putNumber("CLimber Front Selected Sensor postion", m_frontClimber.getSelectedSensorPosition());
  }

  public void startClimbMotor(int direction, double speed) {
    m_frontClimber.set(ControlMode.PercentOutput, direction*speed);
  }

  public void stopClimbMotor() {
    m_frontClimber.neutralOutput();
  }
}
