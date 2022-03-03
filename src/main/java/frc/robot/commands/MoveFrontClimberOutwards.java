// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.FrontClimber;

public class MoveFrontClimberOutwards extends CommandBase {

  private enum State {
    kWaitForRatchet,
    kRunning
  }

  private final FrontClimber m_climber;
  private final Timer m_timer = new Timer();
  private State m_state;
  /** Creates a new ForwardClimbClockwise. */
  public MoveFrontClimberOutwards(FrontClimber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    m_state = State.kWaitForRatchet;

    addRequirements(m_climber);
  }
  
  @Override
  public void initialize() {
    m_state = State.kWaitForRatchet;
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.engageRatchet();
    switch (m_state) {
      case kWaitForRatchet:
        if (m_timer.advanceIfElapsed(0.25)) {
          m_state = State.kRunning;
        }
        
        break;
      case kRunning:
        m_climber.startOutwardClimb();
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.releaseRatchet();
    m_climber.stop();
  }
}
