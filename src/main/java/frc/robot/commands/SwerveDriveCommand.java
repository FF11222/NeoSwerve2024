package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

import static java.lang.Math.abs;
import static java.lang.Math.pow;
<<<<<<< HEAD:NeoSwerve2024/src/main/java/frc/robot/commands/SwerveDriveCommand.java
=======

>>>>>>> origin/master:src/main/java/frc/robot/commands/SwerveDriveCommand.java


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
        double xSpeed = (MathUtil.applyDeadband(-controller.getLeftY(), DriveConstants.DEAD_BAND));
        double ySpeed = MathUtil.applyDeadband(-controller.getLeftX(), DriveConstants.DEAD_BAND);
        double rotation = MathUtil.applyDeadband(-controller.getRightX(), DriveConstants.DEAD_BAND);

        this.subsystem.drive(
                xSpeedLimiter.calculate(xSpeed) * DriveConstants.MAX_SPEED,
                ySpeedLimiter.calculate(ySpeed) * DriveConstants.MAX_SPEED,
                rotateLimiter.calculate(rotation) * DriveConstants.MAX_ANGULAR_SPEED,
                true
        );
    }

    private double speedACurve(double speed) {
        return speed * pow(abs(speed), 2);
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
