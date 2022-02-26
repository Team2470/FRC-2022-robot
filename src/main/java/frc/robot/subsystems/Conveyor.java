// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Conveyor extends SubsystemBase {
  /** Creates a new Conveyer. */
  private final WPI_TalonSRX m_conveyorMotor;
  private final DigitalInput m_firstCargoSensor;
  private final DigitalInput m_secondCargoSensor;

  public Conveyor() {
    m_conveyorMotor = new WPI_TalonSRX(Constants.kConveyorMotorID);
    m_firstCargoSensor = new DigitalInput(Constants.kConveyorFirstCargoChannel);
    m_secondCargoSensor = new DigitalInput(Constants.kConveyorSecondCargoChannel);
  }

  public WPI_TalonSRX getMotor() { return this.m_conveyorMotor; }

  public void up(){
    m_conveyorMotor.set(ControlMode.PercentOutput, -0.5);
  }

  public void down(){
    m_conveyorMotor.set(ControlMode.PercentOutput, 0.5);
  }

  public void stop() {
    m_conveyorMotor.stopMotor();
  }

  public boolean isFull() {
      return (isFirstCargoDetected() && isSecondCargoDetected());
  }

  public int capturedCargoCount() {
    if(isFirstCargoDetected() && !isSecondCargoDetected()){
      return 1;
    }
    else if(isFirstCargoDetected() && isSecondCargoDetected()){
      return 2;
    }
    return 0;
  }

  public boolean isFirstCargoDetected(){

    return !m_firstCargoSensor.get();
  }

  public boolean isSecondCargoDetected(){
    return !m_secondCargoSensor.get();
  }

  @Override
  public void periodic() {
      SmartDashboard.putBoolean("Conveyor First Cargo Detected", isFirstCargoDetected());
      SmartDashboard.putBoolean("Conveyor Second Cargo Detected", isSecondCargoDetected());
      SmartDashboard.putBoolean("Conveyor Full", isFull());
      SmartDashboard.putNumber("Conveyor Cargo Count", capturedCargoCount());
  }
}