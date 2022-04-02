package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.IO;
import frc.robot.RobotContainer;

public class GatherManual extends CommandBase {

    public GatherManual() {
        addRequirements(RobotContainer.FEEDER, RobotContainer.SHOOTER, RobotContainer.GATHER);
    }

    @Override
    public void initialize() {
        RobotContainer.GATHER.lower();
    }

    @Override
    public void execute() {
        double speed = 0.5 * (IO.Axis.DriverRightTrigger.get() - IO.Axis.DriverLeftTrigger.get());

        var proximity = RobotContainer.FEEDER.COLOR_SENSOR.getProximity();
        boolean inProximity = proximity > 400;

        if (!inProximity || speed < 0.0) {
            RobotContainer.GATHER.run(speed);
            RobotContainer.FEEDER.run(speed);
        } else {
            RobotContainer.GATHER.run(speed * 0.85);
            RobotContainer.FEEDER.stop();
        }
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
