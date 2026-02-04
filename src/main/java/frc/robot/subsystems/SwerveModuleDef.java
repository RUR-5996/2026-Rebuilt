package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.SwerveConstants;

public class SwerveModuleDef {

    public TalonFX driveMotor;
    public TalonFX steerMotor;

    //private TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    //private TalonFXConfiguration steerConfig = new TalonFXConfiguration();
    
    // Create Control Requests once to avoid garbage collection pressure
    private final VelocityVoltage m_velocitySetter = new VelocityVoltage(0);
    private final PositionVoltage m_positionSetter = new PositionVoltage(0);

    // constants for conversions
    double METERS_PER_WHEEL_ROTATION = 2 * Math.PI * SwerveConstants.WHEEL_RADIUS_METERS;
    double M_DRIVE_ROTATIONS_PER_METER = 1 / METERS_PER_WHEEL_ROTATION;

    public SwerveModuleDef(TalonFX steerMotor, TalonFX driveMotor, boolean steerInverted, InvertedValue driveInverted) {
        this.steerMotor = steerMotor;
        this.driveMotor = driveMotor;
        
        moduleInit(steerInverted, driveInverted);
    }

    public void moduleInit(boolean steerInverted, InvertedValue driveInverted) {
        // --- Drive Motor Configuration ---
        var driveConfigs = new TalonFXConfiguration();
        
        driveConfigs.Slot0.kP = SwerveConstants.driveKP;
        driveConfigs.Slot0.kI = SwerveConstants.driveKI;
        driveConfigs.Slot0.kD = SwerveConstants.driveKD;

        driveConfigs.CurrentLimits.StatorCurrentLimit = SwerveConstants.CURRENT_LIMIT;
        driveConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        
        driveConfigs.Feedback.SensorToMechanismRatio = SwerveConstants.DRIVE_MOTOR_GEARING;
        driveConfigs.MotorOutput.Inverted = driveInverted;
        driveConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        driveMotor.getConfigurator().apply(driveConfigs);

        // --- Steer Motor Configuration ---
        var steerConfigs = new TalonFXConfiguration();

        steerConfigs.Slot0.kP = SwerveConstants.steerKP;
        steerConfigs.Slot0.kI = SwerveConstants.steerKI;
        steerConfigs.Slot0.kD = SwerveConstants.steerKD;

        steerConfigs.CurrentLimits.StatorCurrentLimit = SwerveConstants.CURRENT_LIMIT;
        steerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        // use SensorToMechanismRatio for gearing
        steerConfigs.Feedback.SensorToMechanismRatio = SwerveConstants.STEER_MOTOR_GEARING;
        steerConfigs.MotorOutput.Inverted = steerInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        steerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steerConfigs.ClosedLoopGeneral.ContinuousWrap = true;

        steerMotor.getConfigurator().apply(steerConfigs);
    }

    public void resetSteerEncoder() {
        // Reset position
        steerMotor.setPosition(0);
    }

    public SwerveModulePosition getModulePosition() {
        // Returns position in Meters and Rotation2d
        return new SwerveModulePosition(
            driveMotor.getPosition().getValueAsDouble() * METERS_PER_WHEEL_ROTATION, 
            Rotation2d.fromRotations(steerMotor.getPosition().getValueAsDouble())
        );
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
            driveMotor.getVelocity().getValueAsDouble() * METERS_PER_WHEEL_ROTATION, 
            Rotation2d.fromRotations(steerMotor.getPosition().getValueAsDouble())
        );
    }

    public void setState(SwerveModuleState desiredState) {
        // 1. Get current angle from the motor
        Rotation2d currentAngle = Rotation2d.fromRotations(steerMotor.getPosition().getValueAsDouble());

        // 2. Optimize
        desiredState.optimize(currentAngle);

        // 3. (Optional) Use the new 2026 cosineScale for smoother driving
        // This scales speed by the cosine of the error to reduce "jitter"
        desiredState.cosineScale(currentAngle);

        // 4. Apply the optimized values to the motors
        steerMotor.setControl(m_positionSetter.withPosition(desiredState.angle.getRotations()));

        double velocityRotationsPerSecond = desiredState.speedMetersPerSecond * M_DRIVE_ROTATIONS_PER_METER;
        driveMotor.setControl(m_velocitySetter.withVelocity(velocityRotationsPerSecond));

        // Debugging
        SmartDashboard.putNumber("Module Angle Error", desiredState.angle.getDegrees() - currentAngle.getDegrees());
    }

    public void setDriveNeutralMode(NeutralModeValue mode) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        // We "refresh" so we don't overwrite other existing settings
        driveMotor.getConfigurator().refresh(config);
        config.MotorOutput.NeutralMode = mode;
        driveMotor.getConfigurator().apply(config);
    }
    
    public void setSteerNeutralMode(NeutralModeValue mode) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        steerMotor.getConfigurator().refresh(config);
        config.MotorOutput.NeutralMode = mode;
        steerMotor.getConfigurator().apply(config);
    }

    // Debugging
    public void forceSteerRotation(double rotationsToSpin) {
        // 1. Get current position in rotations
        double currentRotations = steerMotor.getPosition().getValueAsDouble();
        
        // 2. Calculate target (Current + Desired)
        double targetRotations = currentRotations + rotationsToSpin;

        // 3. Set the steer motor directly (Bypassing optimization)
        steerMotor.setControl(m_positionSetter.withPosition(targetRotations));
        
        // 4. Ensure drive motor is stopped so the robot doesn't move
        driveMotor.setControl(m_velocitySetter.withVelocity(0));
    }
}