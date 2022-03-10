package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class Shoot extends SequentialCommandGroup {
    
    public Shoot() {
        super(
            RobotContainer.SHOOTER.freeBall(),
            RobotContainer.GATHER.tampBall(),
            RobotContainer.SHOOTER.revFlywheel(Units.rotationsPerMinuteToRadiansPerSecond(3000.0))
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        RobotContainer.SHOOTER.stop();
        RobotContainer.GATHER.stop();
    }
}
