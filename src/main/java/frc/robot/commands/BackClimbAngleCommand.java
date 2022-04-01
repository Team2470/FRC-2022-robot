package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.BackClimber;


public class BackClimbAngleCommand extends CommandBase {
  private final BackClimber m_climber;
  private final Rotation2d m_setpoint;

  public BackClimbAngleCommand(BackClimber climber, Rotation2d setpoint) {
    m_climber = climber;
    m_setpoint = setpoint;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(m_climber);
  }

  public Rotation2d getError() {
    return m_climber.getAngle().minus(m_setpoint);
  }

  @Override
  public void initialize() {
    SmartDashboard.putString("Climber Angle Commant", "init");
    m_climber.enable();
    m_climber.setSetpoint(m_setpoint.getDegrees());
  }

  @Override
  public void execute() {
    SmartDashboard.putString("Climber Angle Commant", "execute");
    SmartDashboard.putNumber("Climber Angle Command Setpoint", m_setpoint.getDegrees());
    m_climber.setSetpoint(m_setpoint.getDegrees());
  }

  @Override
  public boolean isFinished() {
    double threshold = Constants.kBackClimberErrorBound;
    if(m_setpoint.getDegrees() == 90) { threshold = 3.0; }
    return Math.abs(getError().getDegrees()) <= threshold;
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Climber Angle Commant", "end");
    m_climber.stop();
  }
}
