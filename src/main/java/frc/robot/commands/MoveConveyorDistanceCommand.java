// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Conveyor;

public class MoveConveyorDistanceCommand extends CommandBase {
  private final double m_meters;
  private final Conveyor m_conveyor;

  public MoveConveyorDistanceCommand(Conveyor conveyor, double meters) {
    m_meters = meters;
    m_conveyor = conveyor;

    addRequirements(conveyor);
  }

  public double getError() {
    return m_meters - m_conveyor.getDistance();
  }

  @Override
  public void initialize() {
    m_conveyor.zeroDistance();
  }

  @Override
  public void execute() {
    double error = getError();
    if (error > 0) {
      m_conveyor.up();
    } else {
      m_conveyor.down();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_conveyor.stop();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(getError()) <= Constants.kConveyorErrorBoundM;
  }
}
