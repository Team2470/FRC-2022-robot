// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Conveyor extends SubsystemBase {
  /** Creates a new Conveyer. */
  private WPI_TalonSRX m_conveyorMotor;

  public Conveyor() {
    m_conveyorMotor = new WPI_TalonSRX(Constants.kConveyorMotorID);
  }

  public WPI_TalonSRX getMotor() { return this.m_conveyorMotor; }

  public void move(int direction, double speed) {
    m_conveyorMotor.set(ControlMode.PercentOutput, direction*speed);
  }

  public void moveAtSpeed (double speed) {
    m_conveyorMotor.set(ControlMode.PercentOutput, speed);

  }

  public void stop() {
    m_conveyorMotor.stopMotor();
  }


  /**
   * Detects when the second cargo is in the desired position.
   *
   * We need to sill detect the cargo when the conveyor moves it down 1 inch (or so) when it prepares to shooter
   *
   * @return
   */
  public boolean isFirstCargoDetected() {
    return false; // TODO
  }

  /**
   * Detects when the second cargo is in the desired position.
   * @return
   */
  public boolean isSecondCargoDetected() {
    return false; // TODO
  }

  public boolean isFull() {
    return isFirstCargoDetected() && isSecondCargoDetected();
  }

  public int capturedBallCount() {
    if (isFirstCargoDetected() && !isSecondCargoDetected()) {
      return 1;
    } else if (isFirstCargoDetected() && isSecondCargoDetected()) {
      return 2;
    } else {
      return 0;
    }
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Conveyor First Cargo Detected", isFirstCargoDetected());
    SmartDashboard.putBoolean("Conveyor Second Cargo Detected", isSecondCargoDetected());
    SmartDashboard.putNumber("Conveyor Captured Cargo Count", capturedBallCount());
    SmartDashboard.putBoolean("Conveyor Full", isFull());
  }
}