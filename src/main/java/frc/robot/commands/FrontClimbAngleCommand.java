package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.FrontClimber;


public class FrontClimbAngleCommand extends CommandBase {
  private final FrontClimber m_frontClimber;
  private final Rotation2d m_setpoint;

  public FrontClimbAngleCommand(FrontClimber frontClimber, Rotation2d setpoint) {
    m_frontClimber = frontClimber;
    m_setpoint = setpoint;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(m_frontClimber);
  }

  public Rotation2d getError() {
    return m_frontClimber.getAngle().minus(m_setpoint);
  }

  @Override
  public void execute() {
    // TODO: This needs to be adjusted
    if (getError().getDegrees() > 0) {
      m_frontClimber.startOutwardClimb();
    } else {
      m_frontClimber.startInwardClimb();
    }
  }

  @Override
  public boolean isFinished() {
    return Math.abs(getError().getDegrees()) <= Constants.kClimberErrorBound;
  }

  @Override
  public void end(boolean interrupted) {
    m_frontClimber.stop();
  }
}