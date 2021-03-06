package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;


public class RetractIntakeCommand extends CommandBase {
  private final Intake m_intake;

  public RetractIntakeCommand(Intake intake) {
    m_intake = intake;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(m_intake);
  }

  @Override
  public void execute() {
    m_intake.stopIntakeMotor();
    m_intake.raise();
  }
}
