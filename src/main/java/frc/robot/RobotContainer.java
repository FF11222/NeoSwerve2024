// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    private final SwerveSubsystem driveSubsystem = new SwerveSubsystem();
    private final XboxController driveController = new XboxController(0);
    private final SwerveDriveCommand driveCommand = new SwerveDriveCommand(this.driveSubsystem, driveController);

    public RobotContainer() {
        this.driveSubsystem.setDefaultCommand(this.driveCommand);
        configureBindings();
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
