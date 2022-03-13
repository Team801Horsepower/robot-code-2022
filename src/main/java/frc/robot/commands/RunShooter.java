package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class RunShooter extends CommandBase {
    public RunShooter() {
        addRequirements(RobotContainer.SHOOTER);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.SHOOTER.setSpeed(Units.rotationsPerMinuteToRadiansPerSecond(2100.0));
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("RPM Reached", RobotContainer.SHOOTER.FLYWHEEL.velocityReached(Units.rotationsPerMinuteToRadiansPerSecond(50.0)));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.SHOOTER.stop();
    }
}
