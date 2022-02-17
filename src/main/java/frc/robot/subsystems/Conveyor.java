// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Conveyor extends SubsystemBase {
  /** Creates a new Conveyer. */
  private WPI_TalonFX m_conveyorMotor;

  public Conveyor() {
    m_conveyorMotor = new WPI_TalonFX(Constants.kConveyorMotorID);
  }

  public void move(int direction, double speed) {
    m_conveyorMotor.set(ControlMode.PercentOutput, direction*speed);
  }

  public void moveAtSpeed (double speed) {
    m_conveyorMotor.set(ControlMode.PercentOutput, speed);

  }

  public void stop() {
    m_conveyorMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}