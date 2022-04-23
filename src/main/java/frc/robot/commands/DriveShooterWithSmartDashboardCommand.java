package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;


public class DriveShooterWithSmartDashboardCommand extends CommandBase {
  private final Shooter m_shooter;

  public DriveShooterWithSmartDashboardCommand(Shooter shooter) {
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(shooter);
    m_shooter = shooter;
    SmartDashboard.putNumber("Shooter Control RPM", 0);
  }

  @Override
  public void execute() {
    m_shooter.setStateSpaceControlEnabled(true);
    m_shooter.setRPM(SmartDashboard.getNumber("Shooter Control RPM", 0));
  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.setStateSpaceControlEnabled(false);
    m_shooter.stop();
  }
}
