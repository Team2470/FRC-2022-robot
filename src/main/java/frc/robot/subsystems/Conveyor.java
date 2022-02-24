// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Conveyor extends SubsystemBase {
  /** Creates a new Conveyer. */
  private WPI_TalonSRX m_conveyorMotor;

  public Conveyor() {
    m_conveyorMotor = new WPI_TalonSRX(Constants.kConveyorMotorID);
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
// TODO
  public boolean isFull() {
    return false;
  }
// TODO
  public int cargoCount() {
    return 0;
  }

}