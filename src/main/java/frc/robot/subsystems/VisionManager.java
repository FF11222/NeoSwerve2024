    package frc.robot.subsystems;

    import edu.wpi.first.math.util.Units;
    import edu.wpi.first.networktables.NetworkTable;
    import edu.wpi.first.networktables.NetworkTableEntry;
    import edu.wpi.first.networktables.NetworkTableInstance;
    import edu.wpi.first.wpilibj2.command.SubsystemBase;
    import frc.robot.Constants.VisionConstants;

    public class VisionManager extends SubsystemBase {
        private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

        public static double getDistanceToGoalMeters() {
            NetworkTableEntry ty = table.getEntry("ty");
            double targetOffsetAngle_Vertical = ty.getDouble(0.0);

            double angleToGoalRadians = Units.degreesToRadians(
                    VisionConstants.LimelightMountAngleDegrees + targetOffsetAngle_Vertical);

            return (VisionConstants.GoalHeightMeters - VisionConstants.LimelightLensHeightMeters) / Math.tan(angleToGoalRadians);
        }

        public static double getHorizontalDistanceToGoalMeters(double distanceToGoalMeter) {
            if (distanceToGoalMeter == -1) {
                distanceToGoalMeter = getDistanceToGoalMeters();
            }
            NetworkTableEntry tx = table.getEntry("tx");
            double targetOffSetAngle_Horizontal = tx.getDouble(0.0);

            @SuppressWarnings("")
            double distanceToGoalHorizontalMeters = (
                    Math.tan(Units.degreesToRadians(targetOffSetAngle_Horizontal)) * distanceToGoalMeter)
                    - VisionConstants.LimelightHorizontalOffsetMeters;

            return distanceToGoalHorizontalMeters;
        }

        public static double getAprilTagID() {
            NetworkTableEntry entry = table.getEntry("tid");
            return entry.getDouble(0.0);
        }
    }
