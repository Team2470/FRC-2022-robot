package frc.robot.commands;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder.BackendKind;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.BackClimber;
import frc.robot.subsystems.FrontClimber;

import java.time.chrono.MinguoEra;
import java.util.Map;
import java.util.zip.Deflater;

public class ClimbSequenceCommandGroup extends SequentialCommandGroup {
  private static ClimbState m_climbState = ClimbState.kStartingPosition;
  private static BarState m_barState = BarState.kMid;

  public ClimbSequenceCommandGroup(FrontClimber frontClimber, BackClimber backClimber) {
    addCommands(
        new SelectCommand(
            Map.ofEntries(
                    Map.entry(ClimbState.kStartingPosition, new ParallelCommandGroup(
                            new MoveFrontClimberCommand(frontClimber, Rotation2d.fromDegrees(70)),
                            new BackClimbAngleCommand(backClimber, Rotation2d.fromDegrees(45), -Constants.kMaxBackClimberSpeedIn, 0.5)
                    )),
                // Fully auto section
                Map.entry(ClimbState.kSequence1, new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new BackClimbAngleCommand(backClimber, Rotation2d.fromDegrees(90), -Constants.kMaxBackClimberSpeedIn, 0.5),
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(() -> backClimber.getAngle().getDegrees() >= 75),
                                        new MoveFrontClimberCommand(frontClimber, Rotation2d.fromDegrees(10), 0.6)
                                )
                        ),
                  //new MoveFrontClimberCommand(frontClimber, Rotation2d.fromDegrees(10)),
                  new BackClimbAngleCommand(backClimber, Rotation2d.fromDegrees(30)), // Handover
                  new ParallelCommandGroup(
                    new MoveFrontClimberCommand(frontClimber, Rotation2d.fromDegrees(110)),
                    new SequentialCommandGroup(
                      new WaitUntilCommand(() -> frontClimber.getAngle().getDegrees() >= 20),
                      new BackClimbAngleCommand(backClimber, Rotation2d.fromDegrees(75))
                    )
                  ),
                  new BackClimbAngleCommand(backClimber, Rotation2d.fromDegrees(30))
                )),
                // Grab bar
                Map.entry(ClimbState.kGrabBar, new MoveFrontClimberCommand(frontClimber, Rotation2d.fromDegrees(95))),
                // Pull up
                Map.entry(ClimbState.kPullUp, new SequentialCommandGroup(
                  new BackClimbAngleCommand(backClimber, Rotation2d.fromDegrees(140)),
                  new BackClimbAngleCommand(backClimber, Rotation2d.fromDegrees(125)),
                  new ParallelCommandGroup(
                      new MoveFrontClimberCommand(frontClimber, Rotation2d.fromDegrees(10), 0.6),
                      new BackClimbAngleCommand(backClimber, Rotation2d.fromDegrees(90)))
                  )
                ),
                // Drop traverse
                Map.entry(ClimbState.kDropTraverse, new BackClimbAngleCommand(backClimber, Rotation2d.fromDegrees(135))),
                //Finish on traverse
                Map.entry(ClimbState.kDone, new PrintCommand("Climb Done"))
            ),
            () -> m_climbState
        ).andThen(() -> advanceState())
    );
  }

  public static void reset() {
    m_climbState = ClimbState.kStartingPosition;
    m_barState = BarState.kMid;
    SmartDashboard.putString("Arm State", m_climbState.name());
    SmartDashboard.putString("Bar", m_barState.name());
  }

  public void advanceState() {
    switch (m_climbState) {
      case kStartingPosition:
        m_climbState = ClimbState.kSequence1;
        break;
      case kSequence1:
        m_climbState = ClimbState.kGrabBar;
        break;
      case kGrabBar: 
        switch (m_barState) {
          case kMid:
            m_barState = BarState.kHigh;
            m_climbState = ClimbState.kPullUp;
            break;
          case kHigh:
            m_barState = BarState.kTraverse;
            m_climbState = ClimbState.kDropTraverse;
            break;
          case kTraverse:
            break;
        }
        break;
      case kPullUp:
        m_climbState = ClimbState.kSequence1;
        break;
      case kDropTraverse:
        m_climbState = ClimbState.kDone;
        break;
      case kDone:
        break;
    }
    SmartDashboard.putString("Arm State", m_climbState.name());
    SmartDashboard.putString("Bar", m_barState.name());
  }

  public ClimbState getClimbState() { return m_climbState; }

  private enum ClimbState {
    kStartingPosition,
    kSequence1, // B30->0
    kGrabBar, // F100
    kDropTraverse,
    kPullUp,   //F10 B90
    kDone
  }

  private enum BarState {
    kMid,
    kHigh,
    kTraverse
  }
}