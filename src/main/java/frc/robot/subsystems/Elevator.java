package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

    private static Elevator ELEVATOR;

    TalonFX elevatorMotor;
    TalonFXConfiguration elevatorConfig;

    ElevatorState elevatorState = ElevatorState.EXTENDED;

    public static Elevator getInstance() {
        if (ELEVATOR == null) {
            ELEVATOR = new Elevator();
        }
        return ELEVATOR;
    }

    public Elevator() {
        elevatorConfig = new TalonFXConfiguration();

        elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
        elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        elevatorConfig.Slot0.kP = ElevatorConstants.ELEVATOR_MOTOR_P;
        elevatorConfig.Slot0.kI = ElevatorConstants.ELEVATOR_MOTOR_I;
        elevatorConfig.Slot0.kD = ElevatorConstants.ELEVATOR_MOTOR_D;
        elevatorConfig.Slot0.kV = ElevatorConstants.ELEVATOR_MOTOR_V;
    }

    public Command extend() {
        return Commands.run(() -> {
            elevatorMotor.setControl(new PositionDutyCycle(ElevatorConstants.ELEVATOR_RETRACTED_POS));
        });
    }

    public Command retract() {
        return Commands.run(() -> {
            elevatorMotor.setControl(new PositionDutyCycle(ElevatorConstants.ELEVATOR_EXTENDED_POS));
        });
    }

    public Command testExtend() {
        return Commands.runOnce(() -> {
            elevatorMotor.set(-0.8);
        });
    }

    public Command testRetract() {
        return Commands.runOnce(() -> {
            elevatorMotor.set(0.8);
        });
    }

    public Command stopExtend() {
        return Commands.runOnce(() -> {
            elevatorMotor.set(0);
        });
    }

    public String getElevatorState() {
        return elevatorState.toString();
    }

    public enum ElevatorState {
        EXTENDED,
        IN_PROCESS,
        RETRACTED
    }
}