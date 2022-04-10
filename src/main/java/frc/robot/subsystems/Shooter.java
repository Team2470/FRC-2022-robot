// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final CANSparkMax m_shooterLeader;
  private final CANSparkMax m_shooterFollower;
  private final RelativeEncoder m_encoder;
  private double m_setpoint = 0;
  private boolean m_stateSpaceControlEnabled = false;

  // State space stuff
  private static final double kFlywheelKv = 0.02083978;
  private static final double kFlywheelKa = 0.0011182;
  // The plant holds a state-space model of our flywheel. This system has the following properties:
  //
  // States: [velocity], in radians per second.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [velocity], in radians per second.
  //
  // The Kv and Ka constants are found using the FRC Characterization toolsuite.
  private final LinearSystem<N1, N1, N1> m_flywheelPlant =
      LinearSystemId.identifyVelocitySystem(kFlywheelKv, kFlywheelKa);

  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N1, N1, N1> m_observer =
      new KalmanFilter<>(
          Nat.N1(),
          Nat.N1(),
          m_flywheelPlant,
          VecBuilder.fill(3.0), // How accurate we think our model is
          VecBuilder.fill(0.01), // How accurate we think our encoder
          // data is
          0.020);

  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N1, N1, N1> m_controller =
      new LinearQuadraticRegulator<>(
          m_flywheelPlant,
          VecBuilder.fill(8.0), // Velocity error tolerance
          VecBuilder.fill(12.0), // Control effort (voltage) tolerance
          0.020);

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  private final LinearSystemLoop<N1, N1, N1> m_loop =
      new LinearSystemLoop<>(m_flywheelPlant, m_controller, m_observer, 12.0, 0.020);

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    m_shooterLeader = new CANSparkMax(Constants.kShooterNeoLeaderId, MotorType.kBrushless);
    m_shooterFollower = new CANSparkMax(Constants.kShooterNeoFollowerId, MotorType.kBrushless);
    m_shooterFollower.follow(m_shooterLeader, true);
    m_shooterFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    m_shooterFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    m_shooterFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);

    m_shooterLeader.setInverted(true);
    m_shooterFollower.setInverted(true);

    m_shooterLeader.setSmartCurrentLimit(40);
    m_shooterFollower.setSmartCurrentLimit(40);
    //m_shooterLeader.enableVoltageCompensation(10);
    //m_shooterFollower.enableVoltageCompensation(10);

    m_encoder = m_shooterLeader.getEncoder();

    m_controller.latencyCompensate(m_flywheelPlant, 0.02, 0.025);
  }

  public void init() {
    m_loop.reset(VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(m_encoder.getVelocity())));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter velocity rpm", m_encoder.getVelocity());
    m_loop.setNextR(VecBuilder.fill(m_setpoint));

    // Correct our Kalman filter's state vector estimate with encoder data.
    double currentVelocity = Units.rotationsPerMinuteToRadiansPerSecond(m_encoder.getVelocity());
    m_loop.correct(VecBuilder.fill(currentVelocity));

    // Update our LQR to generate new voltage commands and use the voltages to predict the next
    // state with out Kalman filter.
    m_loop.predict(0.020);

    // Send the new calculated voltage to the motors.
    // voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    double nextVoltage = m_loop.getU(0);
    if (m_stateSpaceControlEnabled) {
      m_shooterLeader.setVoltage(nextVoltage);
    }


    double error = m_setpoint - currentVelocity;
    SmartDashboard.putNumber("Shooter error", Units.radiansPerSecondToRotationsPerMinute(error));
    SmartDashboard.putNumber("Shooter next voltage", nextVoltage);
    SmartDashboard.putNumber("Shooter velocity", Units.radiansPerSecondToRotationsPerMinute(currentVelocity));
    SmartDashboard.putNumber("Shooter target", Units.radiansPerSecondToRotationsPerMinute(m_setpoint));
    SmartDashboard.putNumber("Shooter % error", (error / currentVelocity) * 100);
  }

  public void setRPM(double setPoint) {
    m_setpoint = Units.rotationsPerMinuteToRadiansPerSecond(setPoint);
  }

  public void setStateSpaceControlEnabled(boolean val) { m_stateSpaceControlEnabled = val; }

  public double getRPM() {
    return m_encoder.getVelocity();
  }
  
  public double getError(){
    double currentVelocity = Units.rotationsPerMinuteToRadiansPerSecond(m_encoder.getVelocity());
    double error = m_setpoint - currentVelocity;
    return (error / currentVelocity) * 100;
  }

  public void setPercent(double percent) {
    m_shooterLeader.set(percent);
  }

  public void stop() {
    m_shooterLeader.stopMotor();
    setRPM(0);
  }
}