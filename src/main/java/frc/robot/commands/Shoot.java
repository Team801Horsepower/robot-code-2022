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
        addRequirements(RobotContainer.SHOOTER);
        addRequirements(RobotContainer.MAGAZINE);
        addRequirements(RobotContainer.GATHERER);

        timer = new Timer();

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.start();
        RobotContainer.SHOOTER.enableShooter();
        RobotContainer.GATHERER.reverse();
        RobotContainer.MAGAZINE.reverse();
        Timer.delay(0.06);
        RobotContainer.MAGAZINE.stop();
        RobotContainer.GATHERER.stop();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (RobotContainer.SHOOTER.isReady()) {
            RobotContainer.MAGAZINE.forward();
            RobotContainer.GATHERER.forward();
            RobotContainer.SHOOTER.popUp();
        } else {
            RobotContainer.MAGAZINE.stop();
            RobotContainer.GATHERER.stop();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.SHOOTER.stop();
        RobotContainer.MAGAZINE.stop();
        RobotContainer.GATHERER.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
