package frc.robot.commands;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BackClimber;
import frc.robot.subsystems.FrontClimber;

import java.util.Map;

public class ClimbSequenceCommandGroup extends SequentialCommandGroup {
  private ClimbState m_climbState = ClimbState.kStartingConfig;
  private BarState m_barState = BarState.kNone;

  public ClimbSequenceCommandGroup(FrontClimber frontClimber, BackClimber backClimber) {
    addCommands(
        new SelectCommand(
            Map.of(
                // Starting Config
                // Front out (65), back out (90)
                ClimbState.kStartingConfig, new ParallelCommandGroup(
                    new MoveFrontClimberCommand(frontClimber, Rotation2d.fromDegrees(65)),
                    new BackClimbAngleCommand(backClimber, Rotation2d.fromDegrees(90))
                ),
                // Pull up
                // Front in (limit)
                ClimbState.kPullUpFront, new MoveFrontClimberCommand(frontClimber, Rotation2d.fromDegrees(0)),
                // Handover
                // Back in (30 -> limit)
                ClimbState.kHandover, new BackClimbAngleCommand(backClimber, Rotation2d.fromDegrees(30)),
                // Swing to clear high bar
                // Front out (65), back out (90)
                ClimbState.kSwingToClear, new ParallelCommandGroup(
                    new MoveFrontClimberCommand(frontClimber, Rotation2d.fromDegrees(65)),
                    new BackClimbAngleCommand(backClimber, Rotation2d.fromDegrees(90))
                ),
                // Out to catch high bar
                // Front out (110)
                ClimbState.kReadyToCatch, new MoveFrontClimberCommand(frontClimber, Rotation2d.fromDegrees(110)),
                // Pull up
                // Back in (30 -> limit)
                ClimbState.kPullUpBack, new BackClimbAngleCommand(backClimber, Rotation2d.fromDegrees(30)),
                // Grab high bar
                // Front in (100)
                ClimbState.kGrabBar, new MoveFrontClimberCommand(frontClimber, Rotation2d.fromDegrees(100)),
                // Drop midbar
                // Back out (125)
                ClimbState.kDropBar, new BackClimbAngleCommand(backClimber, Rotation2d.fromDegrees(125)),
                // Pull up
                // Front in (limit), back in (90)
                ClimbState.kPullUp, new ParallelCommandGroup(
                    new MoveFrontClimberCommand(frontClimber, Rotation2d.fromDegrees(10)),
                    new BackClimbAngleCommand(backClimber, Rotation2d.fromDegrees(90))),
                // Handover
                // Back in (30 -> limit)

                // Swing to clear traverse bar
                // Front out (65), back out (90)

                // Out to catch traverse bar
                // Front out (110)

                // Grab traverse bar
                // Front in (100)

                // Drop highbar
                // Back out (125)

                //Finish on traverse
                ClimbState.kDone, new PrintCommand("Climb Done")
            ),
            () -> m_climbState
        )

    );
  }

  public void advanceState() {
    switch (m_climbState) {
      case kStartingConfig:
        m_climbState = ClimbState.kPullUpFront;
        m_barState = BarState.kMid;
        break;
      case kPullUpFront:
        m_climbState = ClimbState.kHandover;
        break;
      case kHandover:
        m_climbState = ClimbState.kSwingToClear;
        break;
      case kSwingToClear:
        m_climbState = ClimbState.kReadyToCatch;
        break;
      case kReadyToCatch:
        m_climbState = ClimbState.kPullUpBack;
        break;
      case kPullUpBack:
        m_climbState = ClimbState.kGrabBar;
        break;
      case kGrabBar:
        m_climbState = ClimbState. kDropBar;
        break;
      case kDropBar:
        switch (m_barState) {
          case kMid:
            m_barState = BarState.kHigh;
            m_climbState = ClimbState.kPullUp;
            break;
          case kHigh:
            m_barState = BarState.kTraverse;
            m_climbState = ClimbState.kDone;
            break;
        }
        break;
      case kPullUp:
        m_climbState = ClimbState.kHandover;
        break;
    }
    SmartDashboard.putString("Arm State", m_climbState.name());
    SmartDashboard.putString("Bar", m_barState.name());
  }

  public ClimbState getClimbState() { return m_climbState; }

  private enum ClimbState {
    kStartingConfig, // F65 B90
    kPullUpFront, // F0, B90
    kHandover, // B30->0
    kSwingToClear, // F65, B90
    kReadyToCatch, // F110
    kPullUpBack, // B30->0
    kGrabBar, // F100
    kDropBar, // B125
    kPullUp,   //F10 B90
    kDone
  }

  private enum BarState {
    kNone,
    kMid,
    kHigh,
    kTraverse
  }
}