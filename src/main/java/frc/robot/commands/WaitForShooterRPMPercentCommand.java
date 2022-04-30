// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

import java.util.function.IntSupplier;

public class WaitForShooterRPMPercentCommand extends CommandBase {
  private final Shooter m_shooter;
  private final IntSupplier m_rpmSupplier;
  private final double m_tolerance;

  /**
   * Creates a new WaitForShooterRPMCommand.
   */
  public WaitForShooterRPMPercentCommand(Shooter shooter, IntSupplier rpmSupplier, double tolerance) {
    m_shooter = shooter;
    m_rpmSupplier = rpmSupplier;
    m_tolerance = tolerance;
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_shooter.getError()) < m_tolerance;
  }
}
