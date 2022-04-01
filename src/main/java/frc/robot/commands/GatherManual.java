package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.IO;
import frc.robot.RobotContainer;

public class GatherManual extends CommandBase {

    public GatherManual() {
        addRequirements(RobotContainer.FEEDER);
        addRequirements(RobotContainer.SHOOTER);
        addRequirements(RobotContainer.GATHER);
    }

    @Override
    public void initialize() {
        RobotContainer.GATHER.lower();
    }

    @Override
    public void execute() {
        double speed = 0.5 * (IO.Axis.DriverRightTrigger.get() - IO.Axis.DriverLeftTrigger.get());
        RobotContainer.SHOOTER.run(-Math.abs(speed));
        RobotContainer.GATHER.run(speed);
        RobotContainer.FEEDER.run(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {
        RobotContainer.SHOOTER.stop();
        RobotContainer.GATHER.stop();
        RobotContainer.FEEDER.stop();
    }
}
