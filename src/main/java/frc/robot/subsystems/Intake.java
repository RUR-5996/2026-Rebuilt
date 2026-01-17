
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{

    private static Intake INTAKE;

    SparkMax intakeFlipOutMotor1;
    RelativeEncoder intakeFlipOutEncoder1;
    SparkClosedLoopController intakeFlipOutController1;
    SparkMax intakeFlipOutMotor2;
    RelativeEncoder intakeFlipOutEncoder2;
    SparkClosedLoopController intakeFlipOutController2;
    TalonFX intakePowerMotor;
    TalonFXConfiguration intakePowerConfig;

    IntakeState intakeState = IntakeState.IN;

    public Intake() {
        
        intakeFlipOutMotor1 = new SparkMax(IntakeConstants.flipOutMotor1Id, MotorType.kBrushless);

        SparkMaxConfig intakeFlipOutConfig = new SparkMaxConfig();
        intakeFlipOutConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        intakeFlipOutConfig.closedLoop
            .p(1.0)
            .i(0.0)
            .d(0.0)
            .positionWrappingEnabled(false);
        intakeFlipOutMotor1.configure(intakeFlipOutConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        intakeFlipOutEncoder1 = intakeFlipOutMotor1.getEncoder();
        intakeFlipOutController1 = intakeFlipOutMotor1.getClosedLoopController();
        intakeFlipOutEncoder1.setPosition(0);

        intakeFlipOutMotor2 = new SparkMax(IntakeConstants.flipOutMotor2Id, MotorType.kBrushless);

        intakeFlipOutConfig
            .inverted(true);
        intakeFlipOutMotor2.configure(intakeFlipOutConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        intakeFlipOutEncoder2 = intakeFlipOutMotor2.getEncoder();
        intakeFlipOutController2 = intakeFlipOutMotor2.getClosedLoopController();
        intakeFlipOutEncoder2.setPosition(0);


        intakePowerMotor = new TalonFX(IntakeConstants.powerMotorId);
        intakePowerConfig = new TalonFXConfiguration();
        intakePowerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakePowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakePowerConfig.Slot0.kP = 1.0;
        intakePowerConfig.Slot0.kI = 0.0;
        intakePowerConfig.Slot0.kD = 0.0;
        intakePowerMotor.getConfigurator().apply(intakePowerConfig);
    }

    public static Intake getInstance() {
        if(INTAKE == null) {
            INTAKE = new Intake();
        }
        return INTAKE;
    }

    public Command intakeFlipOut() {
        if (intakeState == IntakeState.IN){
            intakeState = IntakeState.OUT;
            return Commands.runOnce(() -> {
            moveByRotations(2, intakeFlipOutEncoder1, intakeFlipOutController1);
            moveByRotations(2, intakeFlipOutEncoder2, intakeFlipOutController2);
            intakePowerMotor.set(.8);
            });
        } else {
            return Commands.none();
        } 
    }

    public Command intakeFlipIn() {
        if (intakeState == IntakeState.OUT){
            intakeState = IntakeState.IN;
            return Commands.runOnce(() -> {
            moveByRotations(-2, intakeFlipOutEncoder1, intakeFlipOutController1);
            moveByRotations(-2, intakeFlipOutEncoder2, intakeFlipOutController2);
            intakePowerMotor.stopMotor();
            });
        } else {
            return Commands.none();
        } 
    }

    public String getIntakeState() {
        return intakeState.toString();
    }

    private enum IntakeState {
        IN,
        OUT,
        ERROR,
    }

    public void moveByRotations(double rotations, RelativeEncoder motorEncoder, SparkClosedLoopController motorController)  {
        double currentPosition = motorEncoder.getPosition();
        double targetPosition = currentPosition + rotations;

        motorController.setSetpoint(
            targetPosition,
            ControlType.kPosition
        );
    }
}
