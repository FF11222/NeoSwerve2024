// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    private final SwerveSubsystem driveSubsystem = new SwerveSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final XboxController driveController = new XboxController(0);
    private final XboxController intakeController = new XboxController(1);
    private final SwerveDriveCommand driveCommand = new SwerveDriveCommand(this.driveSubsystem, this.driveController);
    private final IntakeCommand intakeCommand = new IntakeCommand(this.intakeSubsystem, this.intakeController);

    public RobotContainer() {
        this.driveSubsystem.setDefaultCommand(this.driveCommand);
        this.intakeSubsystem.setDefaultCommand(this.intakeCommand);
        configureBindings();
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        PathPlannerPath path = PathPlannerPath.fromPathFile(Constants.AutoConstants.PATH_NAME);
        return AutoBuilder.followPath(path);
    }
}
