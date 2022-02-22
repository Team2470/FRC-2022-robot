package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class WaitForShooterRPMCommand extends CommandBase {

    /**
     *      * Wait for the shooter RPM to be withing the specified tolerance.
     * @param m_shooter
     * @param rpm
     * @param tolerance RPM
     */
    public WaitForShooterRPMCommand(Shooter m_shooter, double rpm, double tolerance) {

    }

    /**
     * Wait for the shooter RPM to be within the default tolerance (100 RPM).
     * @param m_shooter
     * @param rpm
     */
    public WaitForShooterRPMCommand(Shooter m_shooter, double rpm) {
        this(m_shooter, rpm, 100);
    }
}
