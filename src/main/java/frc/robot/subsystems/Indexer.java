package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class Indexer extends SubsystemBase{

    private static Indexer INDEXER;

    TalonFX indexerMotor;
    TalonFXConfiguration indexerConfig;

    private final VelocityVoltage indexerVelocityVoltage = new VelocityVoltage(0);


    public Indexer () {


        indexerMotor = new TalonFX(ShooterConstants.INDEXER_MOTOR_ID);
        indexerConfig = new TalonFXConfiguration();
        indexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        indexerConfig.Slot0.kP = ShooterConstants.INDEXER_MOTOR_P;
        indexerConfig.Slot0.kI = ShooterConstants.INDEXER_MOTOR_I;
        indexerConfig.Slot0.kD = ShooterConstants. INDEXER_MOTOR_D;
        indexerConfig.Slot0.kV = ShooterConstants.INDEXER_MOTOR_V;
        indexerMotor.getConfigurator().apply(indexerConfig);

    }

    public static Indexer getInstance() {
        if(INDEXER == null) {
            INDEXER = new Indexer();
        }
        return INDEXER;
    }

    public Command indexerOn() {
        return Commands.runOnce(() -> {
            indexerMotor.setControl(indexerVelocityVoltage.withVelocity(30));    
        });
    }

    public Command indexerOff() {
        return Commands.runOnce(() -> {
            indexerMotor.stopMotor();
        });
    }
}
