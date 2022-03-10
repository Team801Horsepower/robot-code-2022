package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class GatherBall extends CommandBase {

    public GatherBall() {
        addRequirements(RobotContainer.GATHER);
    }

    @Override
    public void initialize() {
        System.out.println("Gathering");
        RobotContainer.GATHER.lower();
        RobotContainer.GATHER.run(1.0);
    }

    public void end(boolean isInterrupted) {
        System.out.println("Gather Ball End");
        RobotContainer.GATHER.raise();
        RobotContainer.GATHER.stop();
    }
    
}
