package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.components.Neo;

public class Climber extends SubsystemBase {

    private final Neo CLIMB_RIGHT;
    private final Neo CLIMB_LEFT;
    // private final Neo550 CLAW_RIGHT;
    // private final Neo550 CLAW_LEFT;
    
    private final double CLIMB_P = 1.0;
    private final double CLIMB_I = 0.001;
    private final double CLIMB_D = 0.0;

    // private final double CLAW_P = 1.0;
    // private final double CLAW_I = 0.0;
    // private final double CLAW_D = 0.0;

    // double clawSetpoint = 0.0;
    // double climbSetpoint = 0.0;

    public boolean isLocked() {
        return false;
    }

    public Climber() {

        CLIMB_RIGHT = new Neo(Constants.CLIMB_RIGHT);
        CLIMB_RIGHT.setGearRatio(CLIMB_GEAR_RATIO);

        int positionPid = CLIMB_RIGHT.getPositionPid();
        CLIMB_RIGHT.PID.setP(CLIMB_P, positionPid);
        CLIMB_RIGHT.PID.setI(CLIMB_I, positionPid);
        CLIMB_RIGHT.PID.setD(CLIMB_D, positionPid);

        CLIMB_LEFT = new Neo(Constants.CLIMB_LEFT);
        CLIMB_LEFT.setGearRatio(CLIMB_GEAR_RATIO);

        positionPid = CLIMB_LEFT.getPositionPid();
        CLIMB_LEFT.PID.setP(CLIMB_P, positionPid);
        CLIMB_LEFT.PID.setI(CLIMB_I, positionPid);
        CLIMB_LEFT.PID.setD(CLIMB_D, positionPid);

        // CLAW_RIGHT = new Neo550(Constants.CLIMB_RIGHT_CLAW);
        // CLAW_RIGHT.setGearRatio(CLIMB_CLAW_GEAR_RATIO);

        // positionPid = CLAW_RIGHT.getPositionPid();
        // CLAW_RIGHT.PID.setP(CLAW_P, positionPid);
        // CLAW_RIGHT.PID.setI(CLAW_I, positionPid);
        // CLAW_RIGHT.PID.setD(CLAW_D, positionPid);

        // CLAW_LEFT = new Neo550(Constants.CLIMB_LEFT_CLAW);
        // CLAW_LEFT.setGearRatio(CLIMB_CLAW_GEAR_RATIO);

        // positionPid = CLAW_LEFT.getPositionPid();
        // CLAW_LEFT.PID.setP(CLAW_P, positionPid);
        // CLAW_LEFT.PID.setI(CLAW_I, positionPid);
        // CLAW_LEFT.PID.setD(CLAW_D, positionPid);

        CLIMB_RIGHT.setPosition(0.0);
        CLIMB_LEFT.setPosition(0.0);
        SmartDashboard.putData("RIGHT", CLIMB_RIGHT);
        SmartDashboard.putData("LEFT", CLIMB_LEFT);
    }

    public void raiseArm() {
        CLIMB_RIGHT.setDesiredPosition(Constants.CLIMB_PRIME_POSITION);
        CLIMB_LEFT.setDesiredPosition(Constants.CLIMB_PRIME_POSITION);       
    }

    public void driveClimb(double speed) {
        CLIMB_RIGHT.setPower(speed);
        CLIMB_LEFT.setPower(speed);
    }

    // public void driveClaws(double speed) {
    //     clawSetpoint += speed;
    // }

    // public void setClawPosition(double position) {
    //     clawSetpoint = position;
    // }

    public void stop() {
        CLIMB_RIGHT.setPower(0.0);
        CLIMB_LEFT.setPower(0.0);
        // CLIMB_RIGHT.setDesiredPosition(CLIMB_RIGHT.getCurrentPosition());
        // CLIMB_LEFT.setDesiredPosition(CLIMB_LEFT.getCurrentPosition());
        // clawSetpoint = CLAW_RIGHT.getCurrentPosition();
        // climbSetpoint = CLIMB_RIGHT.getCurrentPosition();
    }

    @Override
    public void periodic() {
        // CLAW_LEFT.setDesiredPosition(clawSetpoint);
        // CLAW_RIGHT.setDesiredPosition(clawSetpoint);
        // CLIMB_LEFT.setDesiredPosition(climbSetpoint);
        // CLIMB_RIGHT.setDesiredPosition(climbSetpoint);
    }

    public static final double CLIMB_GEAR_RATIO = 375.0;
    public static final double CLIMB_CLAW_GEAR_RATIO = 371.0;
}
