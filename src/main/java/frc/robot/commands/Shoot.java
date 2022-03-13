package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class Shoot extends SequentialCommandGroup {
    // shooterSpeed in rad/s
    public Shoot(double shooterSpeed) {
        super(
            RobotContainer.GATHER.WHEELS.generatePositionCommand(
                RobotContainer.GATHER.WHEELS.getCurrentPosition() - Math.PI / 4.0,
                0.1,
                RobotContainer.GATHER
            ),
            RobotContainer.SHOOTER.FLYWHEEL.generatePositionCommand(
                RobotContainer.SHOOTER.FLYWHEEL.getCurrentPosition() - Math.PI,
                0.1,
                RobotContainer.SHOOTER
            ),
            RobotContainer.SHOOTER.FLYWHEEL.generateVelocityCommand(
                shooterSpeed,
                1.0,
                RobotContainer.SHOOTER
            ),
            RobotContainer.GATHER.WHEELS.generatePositionCommand(
                RobotContainer.GATHER.WHEELS.getCurrentPosition() + Math.PI / 4.0,
                0.1,
                RobotContainer.GATHER
            ),
            RobotContainer.GATHER.WHEELS.generatePositionCommand(
                RobotContainer.GATHER.WHEELS.getCurrentPosition() - Math.PI / 4.0,
                0.1,
                RobotContainer.GATHER
            ),
            RobotContainer.GATHER.WHEELS.generateVelocityCommand(
                RobotContainer.GATHER.WHEELS.getMaxSpeed(),
                1.0,
                RobotContainer.GATHER
            )
        );
        addRequirements(RobotContainer.GATHER, RobotContainer.SHOOTER);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        RobotContainer.GATHER.stop();
        RobotContainer.SHOOTER.stop();
    }
}
