// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Conveyer extends SubsystemBase {
  /** Creates a new Conveyer. */
  private WPI_TalonFX m_conveyerMotor;

  public Conveyer() {
    m_conveyerMotor = new WPI_TalonFX(Constants.kConveyerMotorID);
  }

  public void move(int direction, double speed) {
    m_conveyerMotor.set(ControlMode.PercentOutput, direction*speed);
  }

  public void stop() {
    m_conveyerMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}