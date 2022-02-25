// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class WaitForShooterRPMCommand extends CommandBase {
  private final Shooter m_shooter;
  private final IntSupplier m_rpmSupplier;
  private final double m_tolerance;

  /** Creates a new WaitForShooterRPMCommand. */
  public WaitForShooterRPMCommand(Shooter shooter, IntSupplier rpmSupplier, double tolerance) {
    m_shooter = shooter;
    m_rpmSupplier = rpmSupplier;
    m_tolerance = tolerance;
  }

  // Convenience constructors
  public WaitForShooterRPMCommand(Shooter shooter, int rpm, double tolerance) {
    this(shooter, () -> rpm, tolerance);
  }
  public WaitForShooterRPMCommand (Shooter shooter, IntSupplier rpmSupplier) {
    this(shooter, rpmSupplier, 100);
  }
  public WaitForShooterRPMCommand (Shooter shooter, int rpm) {
    this(shooter, () -> rpm, 100);
  }

  @Override
  public boolean isFinished() {
    double rpm = m_shooter.getRPM();
    double error = Math.abs(rpm - m_rpmSupplier.getAsInt());
    return error < m_tolerance;
  }
}
