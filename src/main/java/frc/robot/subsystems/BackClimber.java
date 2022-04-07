// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BackClimber extends PIDSubsystem implements Climber {

  private final WPI_TalonFX m_backClimber;
  private final CANCoder m_backCanCoder;
  private static final double kP = 0.06;
  private static final double kI = 0.01;
  private static final double kD = 0.0;
  private double m_maxInwardSpeed = Constants.kMaxBackClimberSpeedIn;
  private double m_maxOutwardSpeed = Constants.kMaxBackClimberSpeedOut;

  /**
   * Creates a new Climber.
   */
  public BackClimber() {
    super(new PIDController(kP, kI, kD));
    getController().setIntegratorRange(0, 0.2);

    m_backClimber = new WPI_TalonFX(Constants.kBackClimberTalonId, Constants.kCanivoreName);
    m_backCanCoder = new CANCoder(Constants.kBackCanCoderId, Constants.kCanivoreName);


    m_backCanCoder.configFactoryDefault();
    m_backCanCoder.configMagnetOffset(Constants.kBackClimberCanCoderOffset);
    m_backCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_backCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    m_backCanCoder.configSensorDirection(true);

    m_backClimber.configFactoryDefault();
    m_backClimber.configForwardSoftLimitEnable(true);
    m_backClimber.configReverseSoftLimitEnable(true);
    m_backClimber.configReverseSoftLimitThreshold(Constants.kBackClimberReverseLimit);
    m_backClimber.configForwardSoftLimitThreshold(Constants.kBackClimberForwardLimit);
    m_backClimber.setNeutralMode(NeutralMode.Brake);

    m_backClimber.configRemoteFeedbackFilter(m_backCanCoder, 0);
    m_backClimber.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
    m_backClimber.setSensorPhase(true);

    m_backClimber.setInverted(true);

    m_backClimber.configVoltageCompSaturation(10);
    m_backClimber.enableVoltageCompensation(true);
    

    // PID is by default disabled
    disable();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
    SmartDashboard.putNumber("Climber Back Angle", m_backCanCoder.getPosition());
    SmartDashboard.putNumber("Climber Back Absolute Angle", m_backCanCoder.getAbsolutePosition());
    SmartDashboard.putNumber("Climber Back Selected Sensor position", m_backClimber.getSelectedSensorPosition());
    SmartDashboard.putNumber("Back Climber Stator Current", m_backClimber.getStatorCurrent());
    SmartDashboard.putNumber("Back Climber Supply Currnet", m_backClimber.getSupplyCurrent());
    SmartDashboard.putNumber("Climber Back Error", getController().getPositionError());
  }

  public void startOutwardClimb() {
    m_backClimber.set(ControlMode.PercentOutput, Constants.kBackClimberSpeed);
  }

  public void startInwardClimb() {
    m_backClimber.set(ControlMode.PercentOutput, -Constants.kBackClimberSpeed);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(this.m_backCanCoder.getAbsolutePosition());
  }

  public void stop() {
    m_backClimber.neutralOutput();
  }

  public void resetSpeed() {
    m_maxOutwardSpeed = Constants.kMaxBackClimberSpeedOut;
    m_maxInwardSpeed = Constants.kMaxBackClimberSpeedIn;
  }

  public void setSpeed(double in, double out) {
    m_maxInwardSpeed = in;
    m_maxOutwardSpeed = out;
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    // TODO Auto-generated method stub

    if(output < m_maxInwardSpeed){
      output = Math.copySign(m_maxInwardSpeed, output);
    } else if(output > m_maxOutwardSpeed){
      output = Math.copySign(m_maxOutwardSpeed, output);
    }

    // If we're targetting 90 degrees, stop within 1 degree
    if (Math.abs(setpoint - 90) <= 1) {
      double error = getAngle().minus(Rotation2d.fromDegrees(setpoint)).getDegrees();
      if (Math.abs(error) <= 3) {
        output = 0;
      }
    }

    m_backClimber.set(ControlMode.PercentOutput, output);
    SmartDashboard.putNumber("Climber Back Output Power", output);
    SmartDashboard.putNumber("Climber Back Setpoint", setpoint);
  }

  @Override
  protected double getMeasurement() {
    // TODO Auto-generated method stub
    return getAngle().getDegrees();
  }
}
