
package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class Shooter extends SubsystemBase{

    SparkMax intakeFlipOutMotor1;
    RelativeEncoder intakeFlipOutEncoder1;
    SparkClosedLoopController intakeFlipOutController1;
    SparkMax intakeFlipOutMotor2;
    RelativeEncoder intakeFlipOutEncoder2;
    SparkClosedLoopController intakeFlipOutController2;

    SparkMax powerMotor1;
    SparkMax powerMotor2;
    SparkMaxConfig powerConfig;

    TalonFX feederMotor;
    TalonFXConfiguration feederConfig;

    CANcoder turretCANcoder;
    SparkMax turretMotor;
    SparkMaxConfig turretConfig;

    private final VelocityVoltage feederVelocityVoltage = new VelocityVoltage(0);


    public Shooter (int powerMotor1Id, int powerMotor2Id, int feederMotorId, int turretCANcoderId, int turretMotorId) {

        powerMotor1 = new SparkMax(powerMotor1Id, MotorType.kBrushless);
        powerMotor2 = new SparkMax(powerMotor2Id, MotorType.kBrushless);
        powerConfig = new SparkMaxConfig();

        powerConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast);
        powerConfig.closedLoop
            .p(1)
            .i(0)
            .d(0);
            

        powerMotor1.configure(powerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        powerConfig.inverted(true);
        powerMotor1.configure(powerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        feederMotor = new TalonFX(feederMotorId);
        feederConfig = new TalonFXConfiguration();
        feederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        feederConfig.Slot0.kP = 1.0;
        feederConfig.Slot0.kI = 0.0;
        feederConfig.Slot0.kD = 0.0;
        feederConfig.Slot0.kV = 0.12;
        feederMotor.getConfigurator().apply(feederConfig);

        turretCANcoder = new CANcoder(turretCANcoderId);
        turretMotor = new SparkMax(turretMotorId, MotorType.kBrushless);

        turretConfig = new SparkMaxConfig();
        
        turretConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);

        turretMotor.configure(turretConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command shooterOn() {
            return Commands.runOnce(() -> {
            powerMotor1.set(1.0);
            powerMotor2.set(1.0);
            });
        }

    public Command shooterOff() {
        return Commands.runOnce(() -> {
            powerMotor1.set(0.0);
            powerMotor2.set(0.0);
        });  
    }

    public Command feederOn() {
        return Commands.runOnce(() -> {
            feederMotor.setControl(feederVelocityVoltage.withVelocity(50));    
        });
    }

    public Command feederOff() {
        return Commands.runOnce(() -> {
            feederMotor.stopMotor();
        });
    }
}
