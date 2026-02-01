
package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ExternalEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class Shooter extends SubsystemBase{

    private static Shooter SHOOTER;

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

    SparkMax turretMotor;
    SparkClosedLoopController turretController;
    ExternalEncoderConfig turretEncoderConfig;
    SparkMaxConfig turretConfig;

    CANcoder turretCANcoder;
    CANcoderConfiguration turretCANcoderConfig;

    private final VelocityVoltage feederVelocityVoltage = new VelocityVoltage(0);


    public Shooter () {

        powerMotor1 = new SparkMax(ShooterConstants.POWER_MOTOR_1_ID, MotorType.kBrushless);
        powerMotor2 = new SparkMax(ShooterConstants.POWER_MOTOR_2_ID, MotorType.kBrushless);
        powerConfig = new SparkMaxConfig();

        powerConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast);
        powerConfig.closedLoop
            .p(ShooterConstants.POWER_MOTOR_P)
            .i(ShooterConstants.POWER_MOTOR_I)
            .d(ShooterConstants.POWER_MOTOR_D);
            

        powerMotor1.configure(powerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        powerMotor2.configure(powerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        feederMotor = new TalonFX(ShooterConstants.FEEDER_MOTOR_ID);
        feederConfig = new TalonFXConfiguration();
        feederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        feederConfig.Slot0.kP = ShooterConstants.FEEDER_MOTOR_P;
        feederConfig.Slot0.kI = ShooterConstants.FEEDER_MOTOR_I;
        feederConfig.Slot0.kD = ShooterConstants. FEEDER_MOTOR_D;
        feederConfig.Slot0.kV = ShooterConstants.FEEDER_MOTOR_V;
        feederMotor.getConfigurator().apply(feederConfig);

        turretMotor = new SparkMax(ShooterConstants.TURRET_MOTOR_ID, MotorType.kBrushless);
        turretConfig = new SparkMaxConfig();
        
        turretConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);

        turretMotor.configure(turretConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        turretController = turretMotor.getClosedLoopController();

        turretEncoderConfig = new ExternalEncoderConfig()
            .positionConversionFactor(360.0 / ShooterConstants.MOTOR_TO_TURRET_RATIO) // degrees per rotation
            .velocityConversionFactor(360.0 / ShooterConstants.MOTOR_TO_TURRET_RATIO); // degrees/sec

        turretCANcoder = new CANcoder(ShooterConstants.TURRET_CANCODER_ID);
        turretCANcoderConfig = new CANcoderConfiguration();

        turretCANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        turretCANcoder.getConfigurator().apply(turretCANcoderConfig);

    }

    public static Shooter getInstance() {
        if(SHOOTER == null) {
            SHOOTER = new Shooter();
        }
        return SHOOTER;
    }

    public Command shooterOn() {
            return Commands.runOnce(() -> {
            powerMotor1.set(0.8);
            powerMotor2.set(0.8);
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

    public double getCANcoderAngle () { //everything is in degrees
        double rotationDegrees = turretCANcoder.getAbsolutePosition().getValueAsDouble() * 360;
        return rotationDegrees * ShooterConstants.CANCODER_TO_TURRET_RATIO;
    }

  public Command rotateTurret (double targetAngle) { //everything is in degrees
    return Commands.runOnce(() -> {
        double[] targetAngleArray = new double[]{targetAngle}; //to get around Java's final requirement in enclosing scopes
        if (targetAngle >= ShooterConstants.MAX_TURRET_ANGLE || 
      targetAngle <= ShooterConstants.MIN_TURRET_ANGLE) {
            targetAngleArray[0] %= 360;
        }
        turretController.setSetpoint(targetAngle, ControlType.kPosition);
    });
  }

  public Command shootOut () {
    return new SequentialCommandGroup(feederOn(), shooterOn(), new WaitCommand(5.0), feederOff(), shooterOff());
  }
}
