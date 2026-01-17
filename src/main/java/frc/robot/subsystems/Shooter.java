
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;

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
    TalonFX powerMotor1;
    TalonFX powerMotor2;
    TalonFXConfiguration powerConfig;
    private final VelocityVoltage intakeVelocityRequest = new VelocityVoltage(0);


    public Shooter(int powerMotor1Id, int powerMotor2Id) {


        powerMotor1 = new TalonFX(powerMotor1Id);
        powerMotor2 = new TalonFX(powerMotor2Id);
        powerConfig = new TalonFXConfiguration();
        powerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        powerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        powerConfig.Slot0.kP = 1.0;
        powerConfig.Slot0.kI = 0.0;
        powerConfig.Slot0.kD = 0.0;
        powerConfig.Slot0.kV = 0.12;
        powerMotor1.getConfigurator().apply(powerConfig);
        powerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        powerMotor2.getConfigurator().apply(powerConfig);
    }

    public Command shooterOn() {
            return Commands.runOnce(() -> {
            powerMotor1.setControl(intakeVelocityRequest.withVelocity(50));
            powerMotor2.setControl(intakeVelocityRequest.withVelocity(50));
            });
        }

    public Command shooterOff() {
        return Commands.runOnce(() -> {
        powerMotor1.stopMotor();
        powerMotor2.stopMotor();
        });  
    }
}
