package frc.robot.components;

import frc.robot.Constants;
import frc.robot.utilities.Utils;

interface SpeedMotorConstructor {
    SpeedMotor constructor();
}
interface AngleMotorConstructor {
    AngleMotor constructor();
}

public class SwerveDrive {
    private final static double thetaChassis = Utils.angle(Constants.ROBOT_LENGTH, Constants.ROBOT_WIDTH);
    private final static double[] rotationAngles = {
            thetaChassis - Math.PI / 2, // Angle for first wheel to turn the
            -thetaChassis - Math.PI / 2, // Angle for second wheel " " " " "
            -thetaChassis + Math.PI / 2, // Angle for third wheel " " " " "
            thetaChassis + Math.PI / 2 // Angle for fourth wheel " " " " "
    };

    private SwervePod[] pods;

    public SwerveDrive(SpeedMotorConstructor driveMotorConstructor, AngleMotorConstructor turnMotorConstructor) {
        pods = new SwervePod[4];
        for (int i = 0; i < 4; i++) {
            SwervePod pod = new SwervePod(driveMotorConstructor.constructor(), turnMotorConstructor.constructor());
            pod.init();
            pods[i] = pod;
        }
    }

    public void periodic() {
        for (SwervePod pod : pods) {
            pod.periodic();
        }
    }

    public void driveAt(double speed, double angle, double rotation) {
        double[] translationVector = {angle, speed};

        double[][] rotationVectors = new double[4][2];

        for (int i = 0; i < 4; i++) {
            rotationVectors[i] = new double[] {rotationAngles[i], rotation};
        }

        double[][] podVectors = new double[4][2];

        for (int i = 0; i < 4; i++) {
            podVectors[i] = Utils.addVectors(translationVector, rotationVectors[i]);
        }

        double maxVectorMagnitude = 0;

        for (int i = 0; i < 4; i++) {
            if (podVectors[i][1] > maxVectorMagnitude) {
                maxVectorMagnitude = podVectors[i][1];
            }
        }

        if (maxVectorMagnitude > 1.0) {
            for (int i = 0; i < 4; i++) {
                podVectors[i][1] /= maxVectorMagnitude;
            }
        }

        for (int i = 0; i < 4; i++) {
            if (speed != 0 || rotation != 0) {
                pods[i].setDesiredAngle(podVectors[i][0]);
                pods[i].setDesiredSpeed(podVectors[i][1]);
            } else {
                pods[i].setDesiredSpeed(0);
            }
        }
    }
}
