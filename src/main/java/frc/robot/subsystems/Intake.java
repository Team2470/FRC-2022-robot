package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private CANSparkMax m_intakeMotor;
    private Solenoid m_deploy;

    public Intake() {
        m_intakeMotor = new CANSparkMax(Constants.kIntakeMotorId, MotorType.kBrushless);
        m_deploy = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.kIntakeSolenoid);
    }
}