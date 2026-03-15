package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

    ClimberState climberState = ClimberState.IDLE;

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


    @Override
    public void periodic() {
        autoExtension();
    }

    private double levelToCm(ClimberState level) {
        switch (level) {
            case L1:
                return ClimberConstants.L1_HEIGHT;
            case L2:
                return ClimberConstants.L2_HEIGHT;
            case L3:
                return ClimberConstants.L3_HEIGHT;
            case IDLE:
                return ClimberConstants.IDLE_HEIGHT;
            default:
                return 0;
        }
    }

    private double levelToPosition(ClimberState level) {
        double levelCm = levelToCm(level);
        double wheelCircumference = 2 * ClimberConstants.CLIMBER_WHEEL_RADIUS * Math.PI;
        
        return levelCm / wheelCircumference * ClimberConstants.CLIMBER_GEAR_RATIO;
        
    }

    public Command setState(String stateName) {
        return Commands.runOnce(() -> {
            switch (stateName) {
                case "IDLE":
                    climberState = ClimberState.IDLE;
                case "L1":
                    climberState = ClimberState.L1;
                case "L2":
                    climberState = ClimberState.L2;
                case "L3":
                    climberState = ClimberState.L3;
            }
        });
    }

    public void autoExtension() {
        climberMotor.setPosition(levelToPosition(climberState));
    }

    public String getClimberState() {
        return climberState.toString();
    }

    public enum ClimberState {
        IDLE, 
        L1,
        L2,
        L3
    }
}