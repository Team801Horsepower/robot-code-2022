package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {

    public static final double ARM_LOWERED_POSITION = -1.0;
    public static final double ARM_RAISED_POSITION = 0.1;

    public static final double MAX_ROBOT_DRIVE_SPEED = 1.0;
    public static final double MAX_ROBOT_TURN_SPEED = 1.0;

    public static final double PATH_MAX_VELOCITY = 1.0;
    public static final double PATH_MAX_ACCELERATION = 0.5;
    public static final double PATH_MAX_ANGULAR_VELOCITY = 0.1;
    public static final double PATH_MAX_ANGULAR_ACCELERATION = 3.1415;

    // PIDs

    // PID for the gyro heading.
    public static final double HEADING_P = 0.005;
    public static final double HEADING_I = 0.0;
    public static final double HEADING_D = 0.0;
    public static final double HEADING_OUTPUT_LIMIT_LOW = -1;
    public static final double HEADING_OUTPUT_LIMIT_HIGH = 1;
    public static final double HEADING_MAX_I_OUT = 1;
    public static final double HEADING_OUTPUT_RAMPRATE = 0.1;
    public static final double HEADING_OUTPUT_FILTER = 0;
    public static final double HEADING_SETPOINT_RANGE = 360;

    // Climb PID
    public static final double CLIMB_P = 0.01;
    public static final double CLIMB_I = 0.0;
    public static final double CLIMB_D = 0.0;

    // Claw PID
    public static final double CLAW_P = 0.01;
    public static final double CLAW_I = 0.0;
    public static final double CLAW_D = 0.0;

    // Gather Wheels PID

    // Gather Arm PID

    // Shooter PID

    // HARDWARE CONFIGURATIONS

    // Swerve Pod Motor CAN IDs
    public static final int POD_1_DRIVE = 9; // Right Front
    public static final int POD_1_TURN = 10;
    public static final int POD_1_THROUGHBORE = 1;

    public static final int POD_2_DRIVE = 12; // Left Front
    public static final int POD_2_TURN = 11;
    public static final int POD_2_THROUGHBORE = 2;

    public static final int POD_3_DRIVE = 20; // Left Rear
    public static final int POD_3_TURN = 19;
    public static final int POD_3_THROUGHBORE = 3;

    public static final int POD_4_DRIVE = 1; // Right Rear
    public static final int POD_4_TURN = 2;
    public static final int POD_4_THROUGHBORE = 4;

    // Climber CAN IDs
    public static final int CLIMB_RIGHT = 4;
    public static final int CLIMB_RIGHT_CLAW = 5;
    public static final int CLIMB_LEFT = 17;
    public static final int CLIMB_LEFT_CLAW = 16;
    public static final double CLIMB_GEAR_RATIO = 1.0 / 375.0;
    public static final double CLIMB_CLAW_GEAR_RATIO = 1.0 / 371.0;

    // Gather Constants
    public static final int GATHER_WHEELS = 15;
    public static final double GATHER_WHEELS_GEAR_RATIO = 1.0 / 9.0;

    public static final int GATHER_ARM = 14;
    public static final double GATHER_ARM_GEAR_RATIO = 1.0 / 30.0;

    // Shooter CAN IDs
    public static final int SHOOTER = 7;
    public static final double SHOOTER_GEAR_RATIO = 1.0;

    // Vision
    public static double CAMERA_HEIGHT = Units.inchesToMeters(23.8);
    public static double CAMERA_PITCH = Units.degreesToRadians(22.4);
    public static double CAMERA_HORIZONTAL_FOV = Units.degreesToRadians(55.02);
    public static double CAMERA_VERTICAL_FOV = Units.degreesToRadians(30.15);
    public static int CAMERA_PIXEL_WIDTH = 1280;
    public static int CAMERA_PIXEL_HEIGHT = 720;
    public static double TARGET_HEIGHT = Units.inchesToMeters(67.5);
    public static double TARGET_TAPE_WIDTH = Units.inchesToMeters(2.0);
    // Competition hub radius
    // public static double TARGET_RADIUS = Units.inchesToMeters(26.0 + 11.0 / 16.0);
    // Testing hub radius
    public static double TARGET_RADIUS = Units.inchesToMeters(27.5);
    public static double TARGET_OFFSET_ANGLE = Units.degreesToRadians(9.75);

    public static Translation2d ORIGIN = new Translation2d();
}
