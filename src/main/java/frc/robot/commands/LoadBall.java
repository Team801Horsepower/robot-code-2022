package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class LoadBall extends CommandBase {
    
    public LoadBall() {
        addRequirements(RobotContainer.FEEDER, RobotContainer.GATHER);
    }

    public void initialize() {
        RobotContainer.FEEDER.run(1.0);
        RobotContainer.GATHER.run(1.0);
        System.out.println("Running Gather");
    }

    public boolean isFinished() {
        return RobotContainer.FEEDER.isLoaded();
    }
    
    public void end(boolean interrupted) {
        System.out.println("Stopping Gather");
        RobotContainer.FEEDER.stop();
        RobotContainer.GATHER.stop();
    }
}
