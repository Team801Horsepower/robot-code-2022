package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.components.SwervePod;
import frc.robot.utilities.Utils;

/**
 * Subsystem to control the entire drive base
 */
public class Chassis extends SubsystemBase
{

    //                      ^ Front
    //                      |
    //           ________________________
    //          /                        \
    //          |    1                2   |
    //          |                         |
    //          |                         |
    //          |                         |
    //          |                         |
    //          |                         |
    //          |                         |
    //          |                         |
    //          |    3                4   |
    //          |                         |
    //          \________________________/

    private SwervePod pod1 = new SwervePod(Constants.POD_1_DRIVE, Constants.POD_1_TURN, Constants.POD_FRONT_LEFT);
    private SwervePod pod2 = new SwervePod(Constants.POD_2_DRIVE, Constants.POD_2_TURN, Constants.POD_FRONT_RIGHT);
    private SwervePod pod3 = new SwervePod(Constants.POD_3_DRIVE, Constants.POD_3_TURN, Constants.POD_BACK_LEFT);
    private SwervePod pod4 = new SwervePod(Constants.POD_4_DRIVE, Constants.POD_4_TURN, Constants.POD_BACK_RIGHT);

    private SwervePod[] pods = new SwervePod[] {pod1, pod2, pod3, pod4};

    public Chassis()
    {
        for (SwervePod pod : pods)
        {
            pod.zeroEncoder();
        }
    }

    /**
     * This method will be called once per scheduler run
     */
    @Override
    public void periodic()
    {

        // Always call to process PID for turn motors
        for (SwervePod pod : pods)
        {
            pod.processPod();
        }

        double x_l = RobotContainer.io.getDriverExpoLeftX(1.5); // Translation X
        double y_l = -RobotContainer.io.getDriverExpoLeftY(1.5); // Translation Y
        double x_r = RobotContainer.io.getDriverExpoRightX(1.5); // Rotation (x)

        // Dimensions will change! What are the dimensions of the test chassis!
        // Change in Constants.java
        //Robot dimensions (example)
        //Length = 24 in
        //Width  = 20 in
        //
        //      20
        //________________
        //|              |
        //|              |
        //|              |
        //|              |
        //|              |  24
        //|              |
        //|              |
        //|              |
        //----------------
        // SEE Constants.java


        // Angle from the center of the robot to the top right wheel
        double thetaChassis = Utils.angle(Constants.ROBOT_LENGTH, Constants.ROBOT_WIDTH); // Gets the angle created from the center of the robot to the top right corner

        double magnitude = Utils.limitRange(Utils.magnitude(x_l, y_l), 0, 1); // Magnitude of left joystick movement

        double angle = Utils.normalizeAngle(Utils.angle(x_l, y_l) - Math.PI / 2); // Angle of left joystick

        double rotationMagnitude = x_r; // Magnitude of right joystick sideways movement

        // Angles of rotation of each wheel
        // Each wheel needs to be perpendicular to the angle from the center to it
        double[] rotationAngles = {
            thetaChassis - Math.PI / 2, //  Angle for first wheel to turn the robot clockwise
            -thetaChassis - Math.PI / 2, // Angle for second wheel "  "   "    "       "
            -thetaChassis + Math.PI / 2, // Angle for third wheel  "  "   "    "       "
            thetaChassis + Math.PI / 2   // Angle for fourth wheel "  "   "    "       "
        };

        double[] translationVector = {angle, magnitude}; // Vector that represents the translation of the robot

        // An array of vectors for each wheel for the wheel rotation
        double[][] rotationVectors = new double[4][2];

        // Create a rotation vector for wheel rotation for each one
        for (int i = 0; i < 4; i++)
        {
            rotationVectors[i] = new double[] {rotationAngles[i], rotationMagnitude};
        }

        // An array of vectors for the final movement of each wheel
        double[][] podVectors = new double[4][2];

        // Add the translation and rotation vectors to get the final movement vector
        for (int i = 0; i < 4; i++)
        {
            podVectors[i] = Utils.addVectors(translationVector, rotationVectors[i]);
        }

        double maxVectorMagnitude = 0;

        for (int i = 0; i < 4; i++)
        {
            if (podVectors[i][1] > maxVectorMagnitude)
            {
                maxVectorMagnitude = podVectors[i][1];
            }
        }

        if (maxVectorMagnitude > 1.0)
        {
            for (int i = 0; i < 4; i++)
            {
                podVectors[i][1] /= maxVectorMagnitude;
            }
        }

        // Loop through each swerve pod
        for (int i = 0; i < 4; i++)
        {
            // If we are moving the sticks
            if (magnitude != 0 || rotationMagnitude != 0)
            {
                // Set the angle and speed of each wheel according to the final vectors
                pods[i].setDesiredAngle(podVectors[i][0]);
                pods[i].setDesiredRPM(podVectors[i][1]);
            }
            else // If we are not moving the sticks, set the wheel speed to 0
            {
                pods[i].setDesiredRPM(0);
            }
        }

    }

    /**
     * A function that gives every pod angle
     * @return An array of every angle of every pod in the order specified above
     */
    public double[] getAngles()
    {
        return new double[] {pod1.getCurrentAngle(), pod2.getCurrentAngle(), pod3.getCurrentAngle(), pod4.getCurrentAngle()};
    }
}
