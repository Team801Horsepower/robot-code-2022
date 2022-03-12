package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class GatherAuto extends CommandBase {

    public GatherAuto() {
        addRequirements(RobotContainer.GATHER);
    }
    
    @Override
    public void initialize() {
        RobotContainer.GATHER.lower();
        RobotContainer.GATHER.run(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.GATHER.stop();
    }
}
