/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class Shoot extends CommandBase {

    Timer timer;

    /**
     * Creates a new Shoot.
     */
    public Shoot() {
        addRequirements(RobotContainer.shooter);
        addRequirements(RobotContainer.magazine);
        addRequirements(RobotContainer.gatherer);

        timer = new Timer();

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.start();
        RobotContainer.shooter.enableShooter();
        RobotContainer.gatherer.reverse();
        RobotContainer.magazine.reverse();
        Timer.delay(0.06);
        RobotContainer.magazine.stop();
        RobotContainer.gatherer.stop();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (RobotContainer.shooter.isReady()) {
            RobotContainer.magazine.forward();
            RobotContainer.gatherer.forward();
            RobotContainer.shooter.popUp();
        } else {
            RobotContainer.magazine.stop();
            RobotContainer.gatherer.stop();
            RobotContainer.shooter.stop();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.shooter.stop();
        RobotContainer.magazine.stop();
        RobotContainer.gatherer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
