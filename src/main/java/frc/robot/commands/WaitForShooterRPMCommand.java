// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class WaitForShooterRPMCommand extends CommandBase {
  /** Creates a new WaitForShooterRPMCommand. */
  public WaitForShooterRPMCommand(Shooter shooter, IntSupplier rpmSupplier, double tolerance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);

  }
  public WaitForShooterRPMCommand (Shooter shooter, IntSupplier rpmSupplier) {
    this(shooter, rpmSupplier, 100);
  }

}
