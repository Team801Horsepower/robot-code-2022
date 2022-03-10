package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class RunArms extends CommandBase {

    double speed;
    
    public RunArms(double speed) {
        this.speed = speed;
        addRequirements(RobotContainer.CLIMBER);
    }

    @Override
    public void initialize() {
        RobotContainer.CLIMBER.driveClimb(speed);
    }

    public void end(boolean isInterrupted) {
        RobotContainer.CLIMBER.stop();
    }
}
