package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;


public class IntakeCommand extends Command {
    private final IntakeSubsystem subsystem;
    private final XboxController controller;

    public IntakeCommand(IntakeSubsystem subsystem, XboxController controller) {
        this.subsystem = subsystem;
        this.controller = controller;

        addRequirements(this.subsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double speed = MathUtil.applyDeadband(-this.controller.getLeftY(), Constants.DriveConstants.DEAD_BAND);
        this.subsystem.takeNotes(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
