// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.jni.CANSparkMaxJNI;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final CANSparkMax m_shooterLeader;
  private final CANSparkMax m_shooterFollower;
  /** Creates a new Shooter. */
  public Shooter() {
    m_shooterLeader = new CANSparkMax(Constants.kShooterNeoLeaderId, MotorType.kBrushless);
    m_shooterFollower = new CANSparkMax(Constants.kShooterNeoFollowerId, MotorType.kBrushless);
    m_shooterFollower.follow(m_shooterLeader,true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
