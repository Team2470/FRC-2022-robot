package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;


public class ClimbAngleCommand extends CommandBase {
    private final Climber m_climber;
    private final Rotation2d m_setpoint;

    public ClimbAngleCommand(Climber climber, Rotation2d setpoint) {
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
    public void execute() {
        // TODO: This needs to be adjusted
        if (getError().getDegrees() > 0) {
            m_climber.startOutwardClimb();
        } else {
            m_climber.startInwardClimb();
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(getError().getDegrees()) <= Constants.kClimberErrorBound;
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.stop();
    }
}
