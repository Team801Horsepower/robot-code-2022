package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.IO;
import frc.robot.RobotContainer;

public class Climb extends CommandBase {

    public Climb() {
        addRequirements(RobotContainer.CLIMBER);
    }

    @Override
    public void execute() {
        RobotContainer.CLIMBER.driveClimb(Constants.CLIMB_SPEED * (IO.Axis.ManipulatorRightTrigger.get() - IO.Axis.ManipulatorLeftTrigger.get()));
    }

    public void end(boolean isInterrupted) {
        RobotContainer.CLIMBER.stop();
    }
}
