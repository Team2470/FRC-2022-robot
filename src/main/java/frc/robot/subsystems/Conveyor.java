// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Conveyor extends SubsystemBase {
  /**
   * Creates a new Conveyer.
   */
  private final WPI_TalonSRX m_conveyorMotor;
  private final DigitalInput m_firstCargoSensor;
  private final DigitalInput m_secondCargoSensor;
  private final WPI_CANCoder m_encoder;

  public Conveyor() {
    setName("Conveyor");

    m_conveyorMotor = new WPI_TalonSRX(Constants.kConveyorMotorID);
    m_firstCargoSensor = new DigitalInput(Constants.kConveyorFirstCargoChannel);
    m_secondCargoSensor = new DigitalInput(Constants.kConveyorSecondCargoChannel);
    m_encoder = new WPI_CANCoder(Constants.kConveyorCanCoder);

    addChild("Motor", m_conveyorMotor);
    addChild("First Cargo Sensor", m_firstCargoSensor);
    addChild("Second Cargo Sensor", m_secondCargoSensor);
    addChild("Encoder", m_encoder);

    m_encoder.configFactoryDefault();
    m_encoder.configSensorDirection(false);
    double countToDegree = 360.0 / 4096.0;
    double radianToMeters = 2 * Math.PI * Constants.kConveyorWheelRadiusM;
    double sensorCoefficient = countToDegree * (1 / 360.0) * radianToMeters;
    m_encoder.configFeedbackCoefficient(sensorCoefficient, "m", SensorTimeBase.PerSecond);

    m_conveyorMotor.configRemoteFeedbackFilter(m_encoder, 0);
    m_conveyorMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
    m_conveyorMotor.setSensorPhase(false);

  }

  public WPI_TalonSRX getMotor() {
    return this.m_conveyorMotor;
  }

  public void up() {
    up(0.5);
  }

  public void up(double speed) {
    m_conveyorMotor.set(ControlMode.PercentOutput, -speed);
  }

  public void down() {
    down(0.5);
  }

  public void down(double speed) {
    m_conveyorMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stop() {
    m_conveyorMotor.stopMotor();
  }

  public boolean isFull() {
    return (isFirstCargoDetected() && isSecondCargoDetected());
  }

  public int capturedCargoCount() {
    if (isFirstCargoDetected() && !isSecondCargoDetected()) {
      return 1;
    } else if (isFirstCargoDetected() && isSecondCargoDetected()) {
      return 2;
    }
    return 0;
  }

  public boolean isFirstCargoDetected() {

    return !m_firstCargoSensor.get();
  }

  public boolean isSecondCargoDetected() {
    return !m_secondCargoSensor.get();
  }

  public double getDistance() {
    return m_encoder.getPosition();
  }

  public void zeroDistance() {
    m_encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Conveyor First Cargo Detected", isFirstCargoDetected());
    SmartDashboard.putBoolean("Conveyor Second Cargo Detected", isSecondCargoDetected());
    SmartDashboard.putBoolean("Conveyor Full", isFull());
    SmartDashboard.putNumber("Conveyor Cargo Count", capturedCargoCount());
    SmartDashboard.putNumber("Conveyor Selected Sensor position", m_conveyorMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Conveyor Distance (in.)", Units.metersToInches(m_encoder.getPosition()));
  }
}