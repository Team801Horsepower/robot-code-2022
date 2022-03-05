/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class FieldDriveWithJoysticks extends CommandBase {
    /**
     * Creates a new FieldDriveWithJoysticks.
     */
    public FieldDriveWithJoysticks() {
        addRequirements(RobotContainer.chassis);
    }

    @Override
    public void execute() {
        double x = RobotContainer.io.getDriverExpoLeftX(2.5);
        double y = RobotContainer.io.getDriverExpoLeftY(2.5);

        double mag = x * x + y * y;
        if (mag > 1) {
            x /= mag;
            y /= mag;
        }

        if (RobotContainer.winch.safeToDrive()) {
            RobotContainer.chassis.fieldDrive(-y, x, RobotContainer.io.getDriverExpoRightX(2.5));
        } else {
            RobotContainer.chassis.stop();
        }
    }

    @Override
    public void end(boolean iterrupted) {
        RobotContainer.chassis.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
