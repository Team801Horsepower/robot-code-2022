package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.IO;
import frc.robot.RobotContainer;

public class Gather extends CommandBase {

    private boolean manual = false;
    private double speed = 0.0;

    public Gather(boolean manual) {
        this.manual = manual;
        addRequirements(RobotContainer.FEEDER, RobotContainer.GATHER);
    }

    @Override
    public void initialize() {
        RobotContainer.GATHER.lower();
    }

    @Override
    public void execute() {
        speed = 1.0;
        if (manual) {
            speed *= IO.Axis.DriverRightTrigger.get() - IO.Axis.DriverLeftTrigger.get();
        }

        if (!RobotContainer.FEEDER.isLoaded() || speed < 0.0) {
            RobotContainer.GATHER.run(speed);
            RobotContainer.FEEDER.run(speed);
        } else {
            RobotContainer.GATHER.run(speed * 0.55);
            RobotContainer.FEEDER.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.FEEDER.isLoaded() && (speed > 0.0 && RobotContainer.GATHER.isJammed());
    }

    public void end(boolean interrupted) {
        RobotContainer.GATHER.stop();
        RobotContainer.FEEDER.stop();
    }
}
