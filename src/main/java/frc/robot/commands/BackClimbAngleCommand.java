package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.BackClimber;


public class BackClimbAngleCommand extends CommandBase {
  private final BackClimber m_climber;
  private final Rotation2d m_setpoint;
  private final double m_maxInwardSpeed;
  private final double m_maxOutwardSpeed;

  public BackClimbAngleCommand(BackClimber climber, Rotation2d setpoint, double inSpeed, double outSpeed) {
    m_climber = climber;
    m_setpoint = setpoint;
    m_maxInwardSpeed = inSpeed;
    m_maxOutwardSpeed = outSpeed;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(m_climber);
  }

  public BackClimbAngleCommand(BackClimber climber, Rotation2d setpoint) {
    this(climber, setpoint, Constants.kMaxBackClimberSpeedIn, Constants.kMaxBackClimberSpeedOut);
  }

  public Rotation2d getError() {
    return m_climber.getAngle().minus(m_setpoint);
  }

  @Override
  public void initialize() {
    SmartDashboard.putString("Climber Angle Commant", "init");
    m_climber.enable();
    m_climber.setSpeed(m_maxInwardSpeed, m_maxOutwardSpeed);
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
    m_climber.resetSpeed();
    m_climber.stop();
  }
}
