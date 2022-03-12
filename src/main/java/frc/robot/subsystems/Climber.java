package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.components.Neo;
import frc.robot.components.Neo550;

public class Climber extends SubsystemBase {

    private final Neo CLIMB_RIGHT;
    private final Neo CLIMB_LEFT;
    private final Neo550 CLAW_RIGHT;
    private final Neo550 CLAW_LEFT;

    double CLIMB_P = 1.0;
    double CLIMB_I = 0.001;
    double CLIMB_D = 0.0;

    double clawSetpoint = 0.0;
    double climbSetpoint = 0.0;

    public boolean isLocked() {
        return false;
    }

    public Climber() {

        CLIMB_RIGHT = new Neo(Constants.CLIMB_RIGHT);
        CLIMB_RIGHT.setGearRatio(Constants.CLIMB_GEAR_RATIO);
        CLIMB_RIGHT.setPosition(0.0);

        int positionPid = CLIMB_RIGHT.getPositionPid();
        CLIMB_RIGHT.PID.setP(CLIMB_P, positionPid);
        CLIMB_RIGHT.PID.setI(CLIMB_I, positionPid);
        CLIMB_RIGHT.PID.setD(CLIMB_D, positionPid);

        CLIMB_LEFT = new Neo(Constants.CLIMB_LEFT);
        CLIMB_LEFT.setGearRatio(Constants.CLIMB_GEAR_RATIO);
        CLIMB_LEFT.setPosition(0.0);

        positionPid = CLIMB_LEFT.getPositionPid();
        CLIMB_LEFT.PID.setP(CLIMB_P, positionPid);
        CLIMB_LEFT.PID.setI(CLIMB_I, positionPid);
        CLIMB_LEFT.PID.setD(CLIMB_D, positionPid);

        CLAW_RIGHT = new Neo550(Constants.CLIMB_RIGHT_CLAW);
        CLAW_RIGHT.setGearRatio(Constants.CLIMB_CLAW_GEAR_RATIO);
        CLAW_RIGHT.setPosition(0.0);

        positionPid = CLAW_RIGHT.getPositionPid();
        CLAW_RIGHT.PID.setP(1.0, positionPid);
        CLAW_RIGHT.PID.setI(0.0, positionPid);
        CLAW_RIGHT.PID.setD(0.0, positionPid);

        CLAW_LEFT = new Neo550(Constants.CLIMB_LEFT_CLAW);
        CLAW_LEFT.setGearRatio(Constants.CLIMB_CLAW_GEAR_RATIO);
        CLAW_LEFT.setPosition(0.0);

        positionPid = CLAW_LEFT.getPositionPid();
        CLAW_LEFT.PID.setP(1.0, positionPid);
        CLAW_LEFT.PID.setI(0.0, positionPid);
        CLAW_LEFT.PID.setD(0.0, positionPid);
    }

    public void raiseArm() {
        climbSetpoint = Constants.CLIMB_PRIME_POSITION;        
    }

    public void driveClimb(double speed) {
        climbSetpoint += speed;
    }

    public void driveClaws(double speed) {
        clawSetpoint += speed;
    }

    public void setClawPosition(double position) {
        clawSetpoint = position;
    }

    public void stop() {
        clawSetpoint = CLAW_RIGHT.getCurrentPosition();
        climbSetpoint = CLIMB_RIGHT.getCurrentPosition();
    }

    @Override
    public void periodic() {
        CLAW_LEFT.setDesiredPosition(clawSetpoint);
        CLAW_RIGHT.setDesiredPosition(clawSetpoint);
        CLIMB_LEFT.setDesiredPosition(climbSetpoint);
        CLIMB_RIGHT.setDesiredPosition(climbSetpoint);
    }
}
