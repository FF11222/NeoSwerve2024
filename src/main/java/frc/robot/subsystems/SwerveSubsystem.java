package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.helpers.SwerveHelper;
import frc.lib.helpers.SwerveHelper.ModuleIds;
import frc.robot.Constants.DeviceIDs.Motors;
import frc.robot.Constants.DeviceIDs.Encoders;
import frc.robot.Constants.Offsets;
import frc.robot.Constants.RobotConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            Motors.frontLeftDrive, Motors.frontLeftTurn, Encoders.frontLeft, Offsets.FRONT_LEFT
            , true, true, "Front Left"
    );
    private final SwerveModule frontRight = new SwerveModule(
            Motors.frontRightDrive, Motors.frontRightTurn, Encoders.frontRight, Offsets.FRONT_RIGHT
            , false, true, "Front Right"
    );
    private final SwerveModule backLeft = new SwerveModule(
            Motors.backwardLeftDrive, Motors.backwardLeftTurn, Encoders.backwardLeft, Offsets.BACK_LEFT
            , true, true, "Back Left"
    );
    private final SwerveModule backRight = new SwerveModule(
            Motors.backwardRightDrive, Motors.backwardRightTurn, Encoders.backwardRight, Offsets.BACK_RIGHT
            , false, true, "Back Right"
    );
    private final SwerveDriveKinematics kinematics = SwerveHelper.constructKinematics(
            RobotConstants.TRACE_WIDTH, RobotConstants.WHEEL_BASE
    );

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
            kinematics, this.gyro.getRotation2d(), this.getPosition()
    );

    public SwerveSubsystem() {
        this.gyro.reset();
    }

    public SwerveModulePosition[] getPosition() {
        return new SwerveModulePosition[]{
                this.frontLeft.getPosition(),
                this.frontRight.getPosition(),
                this.backLeft.getPosition(),
                this.backRight.getPosition()
        };
    }

    public SwerveModuleState[] getState() {
        return new SwerveModuleState[]{
                this.frontLeft.getState(),
                this.frontRight.getState(),
                this.backLeft.getState(),
                this.backRight.getState()
        };
    }

    public double getHeading() {
        return Math.IEEEremainder(this.gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return this.odometry.getPoseMeters();
    }

    @Override
    public void periodic() {
        this.odometry.update(this.gyro.getRotation2d(), this.getPosition());
    }

    public void drive(double xSpeed, double ySpeed, double rotation, boolean field) {
        SwerveModuleState[] states = this.kinematics.toSwerveModuleStates(field ?
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, this.gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rotation)
        );

        setModuleState(states);
    }

    public void setModuleState(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, RobotConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);

        this.frontLeft.setDesiredState(states[ModuleIds.FRONT_LEFT.get()]);
        this.frontRight.setDesiredState(states[ModuleIds.FRONT_RIGHT.get()]);
        this.backLeft.setDesiredState(states[ModuleIds.BACK_LEFT.get()]);
        this.backRight.setDesiredState(states[ModuleIds.BACK_RIGHT.get()]);
    }

    public void setAutoModuleState(ChassisSpeeds speeds) {
        SwerveModuleState[] states = this.kinematics.toSwerveModuleStates(speeds);

        this.frontLeft.setDesiredState(states[ModuleIds.FRONT_LEFT.get()]);
        this.frontRight.setDesiredState(states[ModuleIds.FRONT_RIGHT.get()]);
        this.backLeft.setDesiredState(states[ModuleIds.BACK_LEFT.get()]);
        this.backRight.setDesiredState(states[ModuleIds.BACK_RIGHT.get()]);
    }

    public void stopModules() {
        this.frontLeft.stop();
        this.frontRight.stop();
        this.backLeft.stop();
        this.backRight.stop();
    }
}
