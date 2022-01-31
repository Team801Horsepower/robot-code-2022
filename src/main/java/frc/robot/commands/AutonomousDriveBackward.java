// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2019 FIRST. All Rights Reserved. */
// /* Open Source Software - may be modified and shared by FRC teams. The code */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project. */
// /*----------------------------------------------------------------------------*/

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.RobotContainer;

// public class AutonomousDriveBackward extends CommandBase {

// private Timer timer;
// private double lastTime;
// private double distanceDriven;

// private double distance;

// /**
// * Creates a new AutonomousDriveBackward. Units are unknown.
// */
// public AutonomousDriveBackward(double distance) {
// addRequirements(RobotContainer.chassis);
// timer = new Timer();
// this.distance = distance;
// }

// // Called when the command is initially scheduled.
// @Override
// public void initialize() {
// timer.start();
// lastTime = 0;
// distanceDriven = 0;
// }

// // Called every time the scheduler runs while the command is scheduled.
// @Override
// public void execute() {
// RobotContainer.chassis.directionDrive(0.1, Math.PI);
// distanceDriven += RobotContainer.chassis.getSpeed() * (timer.get() - lastTime);
// lastTime = timer.get();
// }

// // Called once the command ends or is interrupted.
// @Override
// public void end(boolean interrupted) {
// RobotContainer.chassis.stop();
// }

// // Returns true when the command should end.
// @Override
// public boolean isFinished() {
// return distanceDriven >= distance;
// }
// }
