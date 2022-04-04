package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class Shoot extends SequentialCommandGroup {

    public Shoot() {
        super(
            new StartShooter().alongWith(new LoadBall()),
            RobotContainer.FEEDER.feed()
        );
        addRequirements(RobotContainer.FEEDER, RobotContainer.SHOOTER, RobotContainer.GATHER);
    }
    
    public void end(boolean interrupted) {
        RobotContainer.SHOOTER.stop();
        RobotContainer.GATHER.stop();
        RobotContainer.FEEDER.stop();
    }
}
