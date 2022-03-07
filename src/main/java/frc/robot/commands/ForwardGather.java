/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ForwardGather extends CommandBase {
    /**
     * Creates a new DriveWithJoysticks.
     */
    public ForwardGather() {
        addRequirements(RobotContainer.GATHERER);
        addRequirements(RobotContainer.MAGAZINE);
    }

    @Override
    public void initialize() {
        RobotContainer.GATHERER.forward();
        RobotContainer.MAGAZINE.forward();
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.GATHERER.stop();
        RobotContainer.MAGAZINE.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
