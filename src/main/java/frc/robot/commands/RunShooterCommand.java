// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class RunShooterCommand extends CommandBase {
  private final Shooter m_shooter;
  private final IntSupplier m_setPointSupplier;
  /** Creates a new RunShooterCommand. */
  public RunShooterCommand(Shooter shooter, IntSupplier setPointSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_setPointSupplier = setPointSupplier;


    addRequirements(m_shooter);
  }

  public RunShooterCommand(Shooter shooter, int setpoint) {
    this(shooter, () -> setpoint);
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    m_shooter.setRPM(m_setPointSupplier.getAsInt());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
  }

}
