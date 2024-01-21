package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.helpers.IDashboardProvider;
import frc.lib.helpers.SwerveHelper;
import frc.lib.helpers.SwerveHelper.ModuleIds;
import frc.robot.Constants;
import frc.robot.Constants.DeviceIDs.Motors;
import frc.robot.Constants.DeviceIDs.Encoders;
import frc.robot.Constants.Offsets;
import frc.robot.Constants.RobotConstants;

public class SwerveSubsystem extends SubsystemBase implements IDashboardProvider {
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
    private final Field2d field = new Field2d();

    public SwerveSubsystem() {
        this.registerDashboard();
        resetGyro();
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetPose,
                this::getRelativeSpeed,
                this::autoDrive,
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(1., 0.0, 0.0), // Translation PID constants
                        new PIDConstants(1., 0.0, 0.0), // Rotation PID constants
                        Constants.AutoConstants.PHYSICAL_MAX_SPEED, // Max module speed, in m/s
                        new Translation2d(RobotConstants.WHEEL_BASE / 2, RobotConstants.TRACE_WIDTH / 2).getNorm(), // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                },
                this
        );
    }

    public void resetGyro() {
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
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

    public ChassisSpeeds getRelativeSpeed() {
        return this.kinematics.toChassisSpeeds(this.getState());
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

    public void resetPose(Pose2d pose) {
        this.odometry.resetPosition(this.gyro.getRotation2d(), this.getPosition(), pose);
    }

    @Override
    public void periodic() {
        this.odometry.update(this.gyro.getRotation2d(), this.getPosition());
        SmartDashboard.putData("field", field);
        field.setRobotPose(this.getPose());
    }

    public void drive(double xSpeed, double ySpeed, double rotation, boolean field) {
        SwerveModuleState[] states = this.kinematics.toSwerveModuleStates(field ?
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, this.gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rotation)
        );

        setModuleState(states);
    }

    public void autoDrive(ChassisSpeeds speeds) {
        ChassisSpeeds targetSpeed = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState states[] = this.kinematics.toSwerveModuleStates(targetSpeed);
        this.setModuleState(states);
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

    @Override
    public void putDashboard() {
        SmartDashboard.putString("Robot Pose", this.getPose().toString());
        SmartDashboard.putString("Robot Speed", this.getRelativeSpeed().toString());
    }
}
