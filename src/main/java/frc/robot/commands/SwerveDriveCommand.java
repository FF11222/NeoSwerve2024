package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

import static java.lang.Math.abs;
import static java.lang.Math.pow;

public class SwerveDriveCommand extends Command {
    private final SwerveSubsystem subsystem;
    private final XboxController controller;
    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter rotateLimiter = new SlewRateLimiter(3.0);

    public SwerveDriveCommand(SwerveSubsystem subsystem, XboxController controller) {
        this.subsystem = subsystem;
        this.controller = controller;

        addRequirements(this.subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double xSpeed = (MathUtil.applyDeadband(-this.controller.getLeftY(), DriveConstants.DEAD_BAND));
        double ySpeed = MathUtil.applyDeadband(-this.controller.getLeftX(), DriveConstants.DEAD_BAND);
        double rotation = MathUtil.applyDeadband(-this.controller.getRightX(), DriveConstants.DEAD_BAND);

        if (this.controller.getBButton()) {
            this.subsystem.resetPose(new Pose2d(2., 7., new Rotation2d()));
        }

        this.subsystem.drive(
                this.xSpeedLimiter.calculate(xSpeed) * DriveConstants.MAX_SPEED,
                this.ySpeedLimiter.calculate(ySpeed) * DriveConstants.MAX_SPEED,
                this.rotateLimiter.calculate(rotation) * DriveConstants.MAX_ANGULAR_SPEED,
                true
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopModules();
    }
}
