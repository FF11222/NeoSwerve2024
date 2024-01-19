package frc.robot;

public class Constants {
    public static final class DriveConstants {
        public static final double kP = 0.0075;
        public static final double kI = 0.00003;
        public static final double kD = 0.00001;
        public static final double DEAD_BAND = 0.05;
        public static final double MAX_SPEED = 0.7;
        public static final double MAX_ANGULAR_SPEED = 1.5;
    }

    public static final class RobotConstants {
        public static final double TRACE_WIDTH = 0.62;
        public static final double WHEEL_BASE = 0.62;
        public static final double GEAR_RATIO = 57.0 / 7.0;
        public static final double WHEEL_RADIUS = 0.0508;
        public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 3.0;
    }

    public static final class DeviceIDs {
        public static final class Motors {
            public static final int frontLeftDrive = 2;
            public static final int frontRightDrive = 6;
            public static final int backwardLeftDrive = 4;
            public static final int backwardRightDrive = 7;
            public static final int frontLeftTurn = 1;
            public static final int frontRightTurn = 5;
            public static final int backwardLeftTurn = 3;
            public static final int backwardRightTurn = 8;
        }
        public static final class Encoders {
            public static final int frontLeft = 9;
            public static final int frontRight = 10;
            public static final int backwardLeft = 11;
            public static final int backwardRight = 12;
        }
    }

    public static final class Offsets {
        public static final double FRONT_LEFT = 121.376953125;
        public static final double FRONT_RIGHT = 90.17578125;
        public static final double BACK_LEFT = -87.890625;
        public static final double BACK_RIGHT = -148.447265625;
    }

    public static final class AutoConstants {
        public static final double PHYSICAL_MAX_SPEED = 1.0; // m/s
        public static final double PHYSICAL_MAX_ACCELERATION = 0.5; // m/s^2
        public static final String PATH_NAME = "New Path";
    }

    public static final class VisionConstants {
        public static final double LimelightLensHeightMeters = 0.42;
        public static final double GoalHeightMeters = 0.74;
        public static final double LimelightMountAngleDegrees = 0;
        public static final double LimelightHorizontalOffsetMeters = 0.275; // lenght from limelight to the center of robot

        public static final double AutoTrackVerticalP = 0.8;
        public static final double AutoTrackVerticalI = 2.0;
        public static final double AutoTrackVerticalD = 0.01;

        public static double AutoTrackRotationP = 0.8;
        public static double AutoTrackRotationI = 2.0;
        public static double AutoTrackRotationD = 0.01;
    }
}
