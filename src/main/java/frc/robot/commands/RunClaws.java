package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class RunClaws extends CommandBase {

    double speed;

    public RunClaws(double speed) {
        this.speed = speed;
        addRequirements(RobotContainer.CLIMBER);
    }

    @Override
    public void initialize() {
        RobotContainer.CLIMBER.driveClaws(speed);
    }

    public void end(boolean isInterrupted) {
        RobotContainer.CLIMBER.stop();
    }
    
}
