package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

public final class Constants {
        public static double DRIVE_P = 0.0005;
        public static double DRIVE_I = 0.0;
        public static double DRIVE_D = 0.0;
        public static double DRIVE_IZ = 0.0;
        public static double DRIVE_FF = 0.000;
        public static double DRIVE_MAX_OUTPUT = 1.0;
        public static double DRIVE_MIN_OUTPUT = -1.0;
        public static boolean DRIVE_INVERT[] = {true, true, true, true};
        public static IdleMode DRIVE_IDLEMODE[] =
                        {IdleMode.kCoast, IdleMode.kCoast, IdleMode.kCoast, IdleMode.kCoast};

        public static int DRIVE_MAX_RPM = 18730;
        public static int DRIVE_MAX_CURRENT_STALL = 40;
        public static int DRIVE_MAX_CURRENT_RUN = 30;
        public static double DRIVE_METERS_PER_ROTATION = 0.6;

        public static double TURN_P = 0.5; // 0.5 gives a little overshoot on the test stand.
        public static double TURN_I = 0.004; // 0.004
        public static double TURN_D = 0.7; // 0.7
        public static double OutputLowLimit = -1;
        public static double OutputHighLimit = 1;
        public static double MaxIOutput = 0.5;
        public static double OutputRampRate = 1;
        public static double OutputFilter = 0;
        public static double SetpointRange = 2 * Math.PI;

        public static boolean TURN_INVERT[] = {false, false, false, false};
        public static IdleMode TURN_IDLEMODE[] =
                        {IdleMode.kBrake, IdleMode.kBrake, IdleMode.kBrake, IdleMode.kBrake};

        public static int TURN_MAX_CURRENT_STALL = 30;
        public static int TURN_MAX_CURRENT_RUN = 20;

        // Better naming scheme for multiple pods
        public static int POD_1_DRIVE = 13; // Left Front
        public static int POD_1_TURN = 9;

        public static int POD_2_DRIVE = 4; // Right Front
        public static int POD_2_TURN = 8;

        public static int POD_3_DRIVE = 16; // Left Rear
        public static int POD_3_TURN = 12;

        public static int POD_4_DRIVE = 1; // Right Rear
        public static int POD_4_TURN = 5;

        public static int POD_FRONT_LEFT = 0;
        public static int POD_FRONT_RIGHT = 1;
        public static int POD_BACK_LEFT = 2;
        public static int POD_BACK_RIGHT = 3;

        public static double ROBOT_LENGTH = 20.5; // inches
        public static double ROBOT_WIDTH = 20.75; // inches
}
