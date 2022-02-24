package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final CANSparkMax m_intakeMotor;
    private final Solenoid m_deploy;

    public Intake() {
        m_intakeMotor = new CANSparkMax(Constants.kIntakeMotorId, MotorType.kBrushless);
        m_deploy = new Solenoid(PneumaticsModuleType.REVPH, Constants.kIntakeSolenoid);

        m_intakeMotor.setSmartCurrentLimit(40);
        m_intakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        m_intakeMotor.setInverted(false);
    }

    public void lower() {
        m_deploy.set(true);
    }

    public void raise() {
        m_deploy.set(false);
    }

    public void spin() {
        m_intakeMotor.set(Constants.kIntakeSpeed);
    }

    public void stopIntakeMotor() {
        m_intakeMotor.stopMotor();
    }
}
