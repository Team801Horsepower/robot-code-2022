package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class GatherAuto extends CommandBase {

    public GatherAuto() {
        addRequirements(RobotContainer.GATHER);
    }
    
    @Override
    public void initialize() {
        RobotContainer.GATHER.run(1.0);
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.GATHER.stop();
    }
}
