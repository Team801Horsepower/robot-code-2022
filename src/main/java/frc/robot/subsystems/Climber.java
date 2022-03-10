package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.components.Neo;
import frc.robot.components.Neo550;

public class Climber extends SubsystemBase {

    Neo CLIMB_RIGHT;
    Neo CLIMB_LEFT;
    Neo550 CLAW_RIGHT;
    Neo550 CLAW_LEFT;

    public boolean isLocked() {
        return false;
    }

    public Climber() {
        CLIMB_RIGHT = new Neo(Constants.CLIMB_RIGHT);
        CLIMB_RIGHT.setGearRatio(Constants.CLIMB_GEAR_RATIO);
        CLIMB_RIGHT.setPosition(0.0);

        int positionPid = CLIMB_RIGHT.getSpeedPid();
        CLIMB_RIGHT.PID.setP(1.0, positionPid);
        CLIMB_RIGHT.PID.setI(0.0, positionPid);
        CLIMB_RIGHT.PID.setD(0.0, positionPid);

        CLIMB_LEFT = new Neo(Constants.CLIMB_LEFT);
        CLIMB_LEFT.setGearRatio(Constants.CLIMB_GEAR_RATIO);
        CLIMB_LEFT.setPosition(0.0);

        positionPid = CLIMB_LEFT.getSpeedPid();
        CLIMB_LEFT.PID.setP(1.0, positionPid);
        CLIMB_LEFT.PID.setI(0.0, positionPid);
        CLIMB_LEFT.PID.setD(0.0, positionPid);

        CLAW_RIGHT = new Neo550(Constants.CLIMB_RIGHT_CLAW);
        CLAW_RIGHT.setGearRatio(Constants.CLIMB_CLAW_GEAR_RATIO);
        CLAW_RIGHT.setPosition(0.0);

        positionPid = CLAW_RIGHT.getSpeedPid();
        CLAW_RIGHT.PID.setP(1.0, positionPid);
        CLAW_RIGHT.PID.setI(0.0, positionPid);
        CLAW_RIGHT.PID.setD(0.0, positionPid);

        CLAW_LEFT = new Neo550(Constants.CLIMB_LEFT_CLAW);
        CLAW_LEFT.setGearRatio(Constants.CLIMB_CLAW_GEAR_RATIO);
        CLAW_LEFT.setPosition(0.0);

        positionPid = CLAW_LEFT.getSpeedPid();
        CLAW_LEFT.PID.setP(1.0, positionPid);
        CLAW_LEFT.PID.setI(0.0, positionPid);
        CLAW_LEFT.PID.setD(0.0, positionPid);

        SmartDashboard.putData("CLAW_LEFT", CLAW_LEFT);
        SmartDashboard.putData("CLIMB_LEFT", CLIMB_LEFT);
    }

    public void driveClimb(double speed) {
        CLIMB_LEFT.setDesiredSpeed(speed * 2 * Math.PI / 20);
        CLIMB_RIGHT.setDesiredSpeed(speed * 2 * Math.PI / 20);
    }

    public void driveClaws(double speed) {
        CLAW_LEFT.setDesiredSpeed(speed * 2 * Math.PI);
        CLAW_RIGHT.setDesiredSpeed(speed * 2 * Math.PI);
    }

    public void stop() {
        CLIMB_LEFT.setPower(0.0);
        CLIMB_RIGHT.setPower(0.0);
        CLAW_LEFT.setPower(0.0);
        CLAW_RIGHT.setPower(0.0);
    }
}
