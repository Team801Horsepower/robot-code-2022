package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.IO;
import frc.robot.RobotContainer;

public class GatherBall extends CommandBase {

    public GatherBall() {
        addRequirements(RobotContainer.GATHER);
        addRequirements(RobotContainer.SHOOTER);
    }

    @Override
    public void initialize() {
        RobotContainer.GATHER.lower();
        RobotContainer.SHOOTER.setSpeed(-10.0);
    }

    @Override
    public void execute() {
        RobotContainer.GATHER.run(0.5 * (IO.Axis.DriverRightTrigger.get() - IO.Axis.DriverLeftTrigger.get()));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {
        RobotContainer.SHOOTER.stop();
        RobotContainer.GATHER.stop();
    }
}
