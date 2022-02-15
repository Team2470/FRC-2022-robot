// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.Constants;

public class RunConveyorCommand extends CommandBase {
  /** Creates a new RunConveyorCommand. */
  private Conveyor m_conveyor;
  public  double conveyorSpeed;
  
  public RunConveyorCommand(Conveyor conveyor, double conveyorSpeed) {
    m_conveyor = conveyor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_conveyor);    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_conveyor.moveAtSpeed(conveyorSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
