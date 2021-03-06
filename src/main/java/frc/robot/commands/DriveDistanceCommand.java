// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveDistanceCommand extends PIDCommand {
  /** Creates a new DriveDistanceCommand. */
  private final Drive m_drive;
  public DriveDistanceCommand(Drive drive, double distance) {
    super(
        // The controller that the command will use
        new PIDController(0.4, 0, 0),
        // This should return the measurement
        drive::getAverageEncoderDistance,
        // This should return the setpoint (can also be a constant)
        distance,
        // This uses the output
        output -> {
          // Use the output here
          // drive.arcadeDrive(output, 0);
          if (Math.abs(output) < 0.15) {
            output = Math.copySign(0.15, output);
          }
          output = Math.min(output, Constants.kMaxAutoDriveSpeed);
          drive.tankDrive(output, output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    m_drive = drive;

    // Configure additional PID options by calling `getController` here.
  }

  @Override
  public void initialize() {
    super.initialize();
    m_drive.resetEncoders();
  }

  @Override
  public void execute() {
    super.execute();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(getController().getPositionError()) < Units.inchesToMeters(3);
  }
}
