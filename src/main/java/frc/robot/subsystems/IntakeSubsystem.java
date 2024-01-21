package frc.robot.subsystems;


import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeviceIDs.Motors;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax leftMotor = new CANSparkMax(Motors.intakeLeft, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(Motors.intakeRight, MotorType.kBrushless);

    public IntakeSubsystem() {
        this.leftMotor.setSmartCurrentLimit(20);
        this.rightMotor.setSmartCurrentLimit(20);

        this.leftMotor.setIdleMode(IdleMode.kBrake);
        this.rightMotor.setIdleMode(IdleMode.kBrake);

        this.leftMotor.restoreFactoryDefaults();
        this.rightMotor.restoreFactoryDefaults();

        this.leftMotor.setInverted(true);
        this.rightMotor.setInverted(false);
    }

    public void takeNotes(double speed) {
        this.leftMotor.set(speed);
        this.rightMotor.set(speed);
    }
}

