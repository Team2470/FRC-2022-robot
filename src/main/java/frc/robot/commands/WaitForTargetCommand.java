package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;

public class WaitForTargetCommand extends CommandBase{
    private final Vision m_vision;

    public WaitForTargetCommand(Vision vision){
        m_vision = vision;
    }

    @Override
    public boolean isFinished(){
        return m_vision.getTargetFound();
    }
}
