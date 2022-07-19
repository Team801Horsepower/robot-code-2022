package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class RotateGather extends CommandBase {
    double angle;

    public RotateGather(double angle) {
        addRequirements(RobotContainer.GATHER);

        this.angle = angle;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        double desiredAngle = RobotContainer.GATHER.WHEELS.getCurrentAngle() + angle;
        RobotContainer.GATHER.WHEELS.setDesiredAngle(desiredAngle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return RobotContainer.GATHER.WHEELS.positionReached(0.1);
    }
}
