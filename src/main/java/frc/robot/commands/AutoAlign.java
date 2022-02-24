// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Shooter;

public class AutoAlign extends CommandBase {
  private final Vision m_vision;
  private final Conveyor m_conveyor;
  /** Creates a new DriveWithGamepadCommand. */
  public AutoAlign(Vision vision, Conveyor conveyor) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_vision = vision;
    m_conveyor = conveyor;


    addRequirements(vision);
    addRequirements(conveyor);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_vision.getTargetFound()) {
      m_conveyor.up();
    }
    

  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }
}
