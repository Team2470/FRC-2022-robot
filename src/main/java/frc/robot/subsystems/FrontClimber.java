// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FrontClimber extends SubsystemBase implements Climber {

  private final WPI_TalonFX m_frontClimber;
  private final WPI_TalonFX m_frontClimberFollower;
  private final CANCoder m_frontCanCoder;
  private final Solenoid m_ratchetSolenoid;

  /** Creates a new Climber. */
  public FrontClimber() {
    m_frontClimberFollower = new WPI_TalonFX(Constants.kFrontClimberFollowerTalonId, Constants.kCanivoreName);
    m_frontClimber = new WPI_TalonFX(Constants.kFrontClimberTalonId, Constants.kCanivoreName);
    m_frontClimberFollower.follow(m_frontClimber);
    m_frontClimberFollower.setInverted(TalonFXInvertType.OpposeMaster);
    m_frontClimberFollower.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 1000);
    m_frontClimberFollower.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1000);

    m_frontCanCoder = new CANCoder(Constants.kFrontCanCoderId, Constants.kCanivoreName);
    m_ratchetSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.kRatchetSolenoid);

    m_frontCanCoder.configFactoryDefault();
    m_frontCanCoder.configMagnetOffset(Constants.kFrontClimberCanCoderOffset);
    m_frontCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_frontCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);


    // m_frontClimber.configFactoryDefault();
    m_frontClimber.configForwardSoftLimitEnable(true);
    m_frontClimber.configReverseSoftLimitEnable(true);
    m_frontClimber.configReverseSoftLimitThreshold(Constants.kFrontClimberReverseLimit);
    m_frontClimber.configForwardSoftLimitThreshold(Constants.kFrontClimberForwardLimit);

    //     try {
    //     Thread.sleep(1000);
    // } catch (InterruptedException e) {
    //     // TODO Auto-generated catch block
    //     e.printStackTrace();
    //}
    m_frontCanCoder.setPosition(m_frontCanCoder.getAbsolutePosition(), 20);
    m_frontClimber.configRemoteFeedbackFilter(m_frontCanCoder, 0);
    m_frontClimber.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Front Angle", m_frontCanCoder.getPosition());
    SmartDashboard.putNumber("Climber Front Absolute Angle", m_frontCanCoder.getAbsolutePosition());
    SmartDashboard.putNumber("Climber Front Selected Sensor position", m_frontClimber.getSelectedSensorPosition());
  }

  public Rotation2d getAngle() { return Rotation2d.fromDegrees(this.m_frontCanCoder.getAbsolutePosition()); }


  // TODO: Needs to be adjusted to match hardware
  public void startOutwardClimb() {
    m_frontClimber.set(ControlMode.PercentOutput, Constants.kClimberSpeed);
  }

  public void startInwardClimb() {
    m_frontClimber.set(ControlMode.PercentOutput, -Constants.kClimberSpeed);
  }

  public void stop() {
    m_frontClimber.neutralOutput();
  }
}
