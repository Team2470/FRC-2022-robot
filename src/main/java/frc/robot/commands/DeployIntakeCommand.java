package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;


public class DeployIntakeCommand extends CommandBase {
    private final Intake m_intake;

    public DeployIntakeCommand(Intake intake) {
        this.m_intake = intake;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.m_intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        this.m_intake.lower();
        this.m_intake.spin();
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.m_intake.stopIntakeMotor();
    }
}
