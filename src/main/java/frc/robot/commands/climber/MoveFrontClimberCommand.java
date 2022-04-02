package frc.robot.commands.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FrontClimber;


public class MoveFrontClimberCommand extends CommandBase {
  private final FrontClimber m_climber;
  private final Rotation2d m_setpoint;
  private final Timer m_timer = new Timer();
  private Direction m_direction;
  private OutwardState m_outwardState;
  public MoveFrontClimberCommand(FrontClimber climber, Rotation2d setpoint) {
    m_direction = Direction.kUnknown;
    m_setpoint = setpoint;
    m_climber = climber;
    m_outwardState = OutwardState.kWaitForRatchet;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    m_outwardState = OutwardState.kWaitForRatchet;
    m_timer.reset();
    m_timer.start();
    if (m_climber.getAngle().minus(m_setpoint).getDegrees() > 0) {
      m_direction = Direction.kInward;
    } else {
      m_direction = Direction.kOutward;
    }
  }

  @Override
  public void execute() {
    SmartDashboard.putString("Move Front Climber Direction", m_direction.toString());
    SmartDashboard.putString("Move Front Climber Ratchet State", m_outwardState.toString());
    SmartDashboard.putNumber("Move Front Climber Error", m_climber.getAngle().minus(m_setpoint).getDegrees());
    switch (m_direction) {
      case kInward:
        m_climber.startInwardClimb();
        break;
      case kOutward:
        m_climber.engageRatchet();
        switch (m_outwardState) {
          case kWaitForRatchet:
            if (m_timer.advanceIfElapsed(0.25)) {
              m_outwardState = OutwardState.kRunning;
            }
            break;
          case kRunning:
            m_climber.startOutwardClimb();
            break;
        }
        break;
    }
  }

  @Override
  public boolean isFinished() {
    switch (m_direction) {
      case kInward:
        return m_climber.getAngle().getDegrees() < m_setpoint.plus(Rotation2d.fromDegrees(1)).getDegrees()
            || m_climber.isAtReverseSoftLimit();
      case kOutward:
        return m_climber.getAngle().getDegrees() > m_setpoint.minus(Rotation2d.fromDegrees(1)).getDegrees()
            || m_climber.isAtForwardSoftLimit();
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_climber.stop();
    m_climber.releaseRatchet();
  }

  private enum OutwardState {
    kWaitForRatchet,
    kRunning,
  }

  private enum Direction {
    kOutward,
    kInward,
    kUnknown
  }
}
