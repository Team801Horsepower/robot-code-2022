package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class PrimeArms extends CommandBase {

    public PrimeArms() {
        addRequirements(RobotContainer.CLIMBER);
    }

    public void initialize() {
        RobotContainer.CLIMBER.raiseArm();
    }
}
