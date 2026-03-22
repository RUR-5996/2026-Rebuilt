package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    
    private static Climber CLIMBER;

    TalonFX climberMotor;
    TalonFXConfiguration climberConfig;

    ClimberState climberState = ClimberState.EXTENDED;

    public static Climber getInstance() {
        if (CLIMBER == null) {
            CLIMBER = new Climber();
        }
        return CLIMBER;
    }

    public Climber() {
        climberMotor = new TalonFX(ClimberConstants.CLIMBER_MOTOR_ID);
        climberConfig = new TalonFXConfiguration();

        climberConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TODO check if correct
        climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;        

        climberConfig.Slot0.kP = ClimberConstants.CLIMBER_MOTOR_P;
        climberConfig.Slot0.kI = ClimberConstants.CLIMBER_MOTOR_I;
        climberConfig.Slot0.kD = ClimberConstants.CLIMBER_MOTOR_D;
        climberConfig.Slot0.kV = ClimberConstants.CLIMBER_MOTOR_V;

        climberMotor.getConfigurator().apply(climberConfig);
    }


    public Command climb() {
        return Commands.runOnce(() -> {
            if (climberState == ClimberState.EXTENDED) {
                climberState = ClimberState.CLIMBING;
                climberMotor.setControl(new PositionDutyCycle(ClimberConstants.CLIMBER_RETRACTED_POS));
                climberState = ClimberState.RETRACTED;
            }
        });
    }

    public Command unclimb() {
        return Commands.runOnce(() -> {
            if (climberState == ClimberState.RETRACTED) {
                climberState = ClimberState.CLIMBING;
                climberMotor.setControl(new PositionDutyCycle(ClimberConstants.CLIMBER_EXTENDED_POS));
                climberState = ClimberState.EXTENDED;
            }
        });
    }

    public String getClimberState() {
        return climberState.toString();
    }

    public enum ClimberState {
        EXTENDED, 
        CLIMBING,
        RETRACTED
    }
}