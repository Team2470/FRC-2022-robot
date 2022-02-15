// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drive;
import frc.robot.Constants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

public class AutoTest extends CommandBase {
  /** Creates a new AutoTest. */
  private Conveyor m_conveyor;
  private Shooter m_Shooter;
  private Drive m_Drive;

  public AutoTest() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    schedule(
      new SequentialCommandGroup(
        new RunShooterCommand(m_Shooter, -2500),
        new WaitCommand(2),
        new ParallelCommandGroup(
          new RunShooterCommand(m_Shooter, -2500)
          
        ),
        new WaitCommand(5),
        new ParallelCommandGroup(
            new RunShooterCommand(m_Shooter, 0)
        )
      )
    );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
