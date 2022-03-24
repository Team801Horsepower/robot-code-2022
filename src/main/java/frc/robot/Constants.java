package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {

    // SETPOINTS
    public static final double CLIMB_PRIME_POSITION = -1.00;
    public static final double CLAW_HIGH_POSITION = 2.369;
    public static final double CLAW_RELEASE_HIGH_POSITION = -0.343;
    public static final double CLAW_TRAVERSAL_POSITION = -2.4;

    public static final double ARM_LOWERED_POSITION = -1.0;
    public static final double ARM_RAISED_POSITION = 0.0;

    // HARDWARE CONFIGURATIONS

    // Climber CAN IDs
    public static final int CLIMB_RIGHT = 4;
    public static final int CLIMB_RIGHT_CLAW = 5;
    public static final int CLIMB_LEFT = 17;
    public static final int CLIMB_LEFT_CLAW = 16;

    // Gather Constants
    public static final int GATHER_WHEELS = 15;
    public static final int GATHER_ARM = 14;

    // Shooter CAN IDs
    public static final int SHOOTER = 7;

    public static final Pose2d GOAL_POSE = new Pose2d(new Translation2d(Units.feetToMeters(27.0), Units.feetToMeters(0.5 * 27.0)), new Rotation2d(Units.degreesToRadians(24.0)));
    public static final Translation2d TARGET_LOCK_POSITION = new Translation2d(-0.1, 4.0);
    public static final double TARGET_HEIGHT = Units.inchesToMeters(8.0 * 12.0 + 7.0 - 2.0);
    public static final double TARGET_TAPE_WIDTH = Units.inchesToMeters(2.0);
    public static final double TARGET_RADIUS = Units.inchesToMeters(26.0 + 11.0 / 16.0);
    public static final double TARGET_OFFSET_ANGLE = Units.degreesToRadians(9.75);
}
