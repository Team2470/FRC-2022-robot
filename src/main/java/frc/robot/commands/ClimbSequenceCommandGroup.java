package frc.robot.commands;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BackClimber;
import frc.robot.subsystems.FrontClimber;

import java.util.Map;

public class ClimbSequenceCommandGroup extends SequentialCommandGroup {
  private static ClimbState m_climbState = ClimbState.kStartingConfig;
  private static BarState m_barState = BarState.kNone;

  public ClimbSequenceCommandGroup(FrontClimber frontClimber, BackClimber backClimber) {
    addCommands(
        new SelectCommand(
            Map.ofEntries(
                // Starting Config
                // Front out (65), back out (90)
                Map.entry(ClimbState.kStartingConfig, new ParallelCommandGroup(
                    new MoveFrontClimberCommand(frontClimber, Rotation2d.fromDegrees(65)),
                    new BackClimbAngleCommand(backClimber, Rotation2d.fromDegrees(90))
                )),
                // Pull up
                // Front in (limit)
                Map.entry(ClimbState.kPullUpFront, new MoveFrontClimberCommand(frontClimber, Rotation2d.fromDegrees(0))),
                // Handover
                // Back in (30 -> limit)
                Map.entry(ClimbState.kHandover, new BackClimbAngleCommand(backClimber, Rotation2d.fromDegrees(30))),
                // Swing to clear bar
                // Front out (65), back out (90)
                Map.entry(ClimbState.kSwingToClear, new ParallelCommandGroup(
                    new MoveFrontClimberCommand(frontClimber, Rotation2d.fromDegrees(65)),
                    new BackClimbAngleCommand(backClimber, Rotation2d.fromDegrees(90))
                )),
                // Out to catch bar
                // Front out (110)
                Map.entry(ClimbState.kReadyToCatch, new MoveFrontClimberCommand(frontClimber, Rotation2d.fromDegrees(110))),
                // Pull up
                // Back in (30 -> limit)
                Map.entry(ClimbState.kPullUpBack, new BackClimbAngleCommand(backClimber, Rotation2d.fromDegrees(30))),
                // Grab bar
                // Front in (100)
                Map.entry(ClimbState.kGrabBar, new MoveFrontClimberCommand(frontClimber, Rotation2d.fromDegrees(100))),
                // Drop bar
                // Back out (125)
                Map.entry(ClimbState.kDropBar, new BackClimbAngleCommand(backClimber, Rotation2d.fromDegrees(125))),
                //Move back out of the way
                //Back in (limit)
                Map.entry(ClimbState.kMoveBack, new BackClimbAngleCommand(backClimber, Rotation2d.fromDegrees(31))),
                // Pull up
                // Front in (limit), back in (90)
                Map.entry(ClimbState.kPullUp, new ParallelCommandGroup(
                    new MoveFrontClimberCommand(frontClimber, Rotation2d.fromDegrees(10)),
                    new BackClimbAngleCommand(backClimber, Rotation2d.fromDegrees(90)))),
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
                Map.entry(ClimbState.kDone, new PrintCommand("Climb Done"))
            ),
            () -> m_climbState
        ).andThen(() -> advanceState())
      
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
            m_climbState = ClimbState.kMoveBack;
            break;
          case kHigh:
            m_barState = BarState.kTraverse;
            m_climbState = ClimbState.kDone;
            break;
        }
        break;
      case kMoveBack:
        m_climbState = ClimbState.kPullUp;
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
    kMoveBack,  //B30
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