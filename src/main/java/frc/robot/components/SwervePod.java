package frc.robot.components;

public class SwervePod {
    private SpeedMotor driveMotor;
    private AngleMotor turnMotor;
    
    private double lastDesiredAngle;
    private boolean flipFlag;
    
    public SwervePod(SpeedMotor driveMotor, AngleMotor turnMotor) {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
    }
    
    public void init() {
        driveMotor.init();
        turnMotor.init();
    }
    
    public void periodic() {
        driveMotor.periodic();
        turnMotor.periodic();
    }
    
    public void setDesiredSpeed(double speed) {
        if (flipFlag) {
            speed = -speed;
        }
        driveMotor.setDesiredSpeed(speed);
    }
    
    public void setDesiredAngle(double angle) {
        double errorAngle = Math.abs(angle - lastDesiredAngle);
        lastDesiredAngle = angle;
        
        if (errorAngle > Math.PI / 2 && errorAngle < 3 * Math.PI / 2) {
            flipFlag = !flipFlag;
        }
        
        if (flipFlag) {
            angle = (angle + Math.PI) % (2 * Math.PI);
        }
        
        turnMotor.setDesiredAngle(angle);
    }
    
    public double getCurrentSpeed() {
        return driveMotor.getCurrentSpeed();
    }
    
    public double getCurrentAngle() {
        return turnMotor.getCurrentAngle();
    }
}
