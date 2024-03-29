package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class CalibrateVision extends CommandBase {

    public CalibrateVision() {
        SmartDashboard.putData(this);
    }

    public void execute() {
        double odometryDistance = RobotContainer.CHASSIS.getCurrentPose().minus(Constants.GOAL_POSE).getTranslation().getNorm();
        SmartDashboard.putNumberArray("CalibrateVision", new double[] {odometryDistance});
    }
    
}
