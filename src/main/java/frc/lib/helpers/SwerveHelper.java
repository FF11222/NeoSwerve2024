package frc.lib.helpers;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class SwerveHelper {
    public enum ModuleIds {
        FRONT_LEFT(0),
        FRONT_RIGHT(1),
        BACK_LEFT(2),
        BACK_RIGHT(3);


        private final int id;


        ModuleIds(int id) {
            this.id = id;
        }

        public int get() {
            return this.id;
        }
    }

    public static SwerveDriveKinematics constructKinematics(double trackWidth, double wheelBase) {
        return new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        );
    }
}
