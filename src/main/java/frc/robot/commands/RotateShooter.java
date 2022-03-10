package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class RotateShooter extends CommandBase {
    double angle;

    public RotateShooter(double angle) {
        addRequirements(RobotContainer.SHOOTER);

        this.angle = angle;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        double desiredAngle = RobotContainer.SHOOTER.FLYWHEEL.getCurrentAngle() + angle;
        RobotContainer.SHOOTER.FLYWHEEL.setDesiredAngle(desiredAngle);
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
        return RobotContainer.SHOOTER.FLYWHEEL.positionReached(0.1);
    }
}
