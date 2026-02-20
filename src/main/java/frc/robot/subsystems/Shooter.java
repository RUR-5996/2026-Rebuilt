
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
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.VelocityVoltage;

import frc.robot.subsystems.SwerveDrive;

public class Shooter extends SubsystemBase{

    private static Shooter SHOOTER;


    SparkMax powerMotor1;
    SparkMax powerMotor2;
    SparkMaxConfig powerConfig;

    RelativeEncoder powerEncoder1;
    SparkClosedLoopController powerController1;
    RelativeEncoder powerEncoder2;
    SparkClosedLoopController powerController2;

    TalonFX feederMotor;
    TalonFXConfiguration feederConfig;

    SparkMax turretMotor;
    SparkClosedLoopController turretController;
    ExternalEncoderConfig turretEncoderConfig;
    SparkMaxConfig turretConfig;

    CANcoder turretCANcoder;
    CANcoderConfiguration turretCANcoderConfig;

    private final VelocityVoltage feederVelocityVoltage = new VelocityVoltage(0);

    private double turretXabs = 0.0;
    private double turretYabs = 0.0;
    private double turretRotAbs = 0.0;
    private double turretRotRel = 0.0;

    private double targetX = 0.0;
    private double targetY = 0.0;
    private double targetDist = 0.0;

    public SwerveDrive SWERVEDRIVE;

    public Shooter () {

        SWERVEDRIVE.getInstance();

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
        powerConfig.encoder
            .positionConversionFactor(1.0 / ShooterConstants.POWER_MOTOR_GEAR_RATIO)
            .velocityConversionFactor(1.0 / ShooterConstants.POWER_MOTOR_GEAR_RATIO);

        powerEncoder1 = powerMotor1.getEncoder();
        powerController1 = powerMotor1.getClosedLoopController();
        powerEncoder1.setPosition(0);
        powerMotor1.configure(powerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        powerConfig.inverted(true);

        powerEncoder2 = powerMotor2.getEncoder();
        powerController2 = powerMotor2.getClosedLoopController();
        powerEncoder2.setPosition(0);
        powerMotor2.configure(powerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        feederMotor = new TalonFX(ShooterConstants.FEEDER_MOTOR_ID);
        feederConfig = new TalonFXConfiguration();
        feederConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
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
            double targetRPM = ShooterConstants.NEO_MAX_RPM * ShooterConstants.DEFAULT_SHOOTER_SPEED;
            powerController1.setSetpoint(targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
            powerController2.setSetpoint(targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
            });
        }

    public Command shooterOn(double speedPercentage) {
            return Commands.runOnce(() -> {
            double targetRPM = ShooterConstants.NEO_MAX_RPM * speedPercentage;
            powerController1.setSetpoint(targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
            powerController2.setSetpoint(targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
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
            feederMotor.setControl(feederVelocityVoltage.withVelocity(ShooterConstants.FEEDER_VELOCITY));    
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

  public Command rotateTurret (double targetAngle) { //everything is in degrees   TODO make conversion do radians
    return Commands.runOnce(() -> {
        double[] targetAngleArray = new double[]{targetAngle}; //to get around Java's final requirement in enclosing scopes
        if (targetAngle >= ShooterConstants.MAX_TURRET_ANGLE || 
      targetAngle <= ShooterConstants.MIN_TURRET_ANGLE) {
            targetAngleArray[0] %= 360;
        }
        turretController.setSetpoint(targetAngle, ControlType.kPosition);
    });
  }


  //autoaim functions


  public void calcTurretXY() {   //position and rotation of robot
    Pose2d pos = SWERVEDRIVE.getPose();
    double phi = ShooterConstants.VEC_TURRET_PHI + pos.getRotation().getRadians();
    turretXabs = ShooterConstants.VEC_TURRET_LEN*Math.cos(phi)+pos.getX();
    turretYabs = ShooterConstants.VEC_TURRET_LEN*Math.sin(phi)+pos.getY();
  }

  public void calcTurretAbsRotation() {
    double deltaX = targetX - turretXabs;
    double deltaY = targetY - turretYabs;
    targetDist = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    if (deltaX >= 0 && deltaY >= 0) {
        turretRotAbs = Math.asin(deltaY/targetDist);
    }   
    else if (deltaX >= 0 && deltaY <= 0) {
        turretRotAbs = Math.asin(deltaY/targetDist);
    }       
    else if (deltaX <= 0 && deltaY >= 0) {
        turretRotAbs = Math.PI/2 + Math.acos(deltaY/targetDist);
    }
    else if (deltaX <= 0 && deltaY <= 0) {
        turretRotAbs = Math.PI/2 + Math.acos(deltaY/targetDist);
    } 
  }

  public void calcTurretRelRotation() { 
    Rotation2d robotRot = SWERVEDRIVE.getPose().getRotation();
    turretRotRel = turretRotAbs - robotRot.getRadians();
  }

  public double clampRot(double rot) {   //everything in radians currently unused
    rot = Math.toDegrees(rot);
    double newRot = rot % 360;
    if (newRot > 180) {
        newRot = newRot - 360;
    }
    else if (newRot < -180) {
        newRot = newRot + 360;
    }
    return Math.toRadians(newRot);
  }

  public void setTarget(double newTargetX, double newTargetY) {
    targetX = newTargetX;
    targetY = newTargetY;
  }

  public Rotation2d getTurretRelRot() {
    return Rotation2d.fromRadians(turretRotRel);
  }

  public Rotation2d getTurretAbsRot() {
    return Rotation2d.fromRadians(turretRotAbs);
  }

  public Pose2d getTurrePosRelRot() {
    return new Pose2d(turretXabs, turretYabs, Rotation2d.fromRadians(turretRotRel));
  }

  public Pose2d getTurrePosAbsRot() {
    return new Pose2d(turretXabs, turretYabs, Rotation2d.fromRadians(turretRotAbs));
  }

  //test functions

  public Command rotateLeft() {
    return Commands.runOnce(() -> {
        turretMotor.set(0.3);
    });
  }

  public Command rotateRight() {
    return Commands.runOnce(() -> {
        turretMotor.set(-0.3);
    });
  }

  public Command rotateStop() {
    return Commands.runOnce(() -> {
        turretMotor.stopMotor();
    });
  }

  public Command shootOut () {
    return new SequentialCommandGroup(feederOn(), shooterOn(), new WaitCommand(5.0), feederOff(), shooterOff());
  }

  public void periodic() {
    calcTurretXY();
    calcTurretAbsRotation();
    calcTurretRelRotation();
  }

  public void report() {
    SmartDashboard.putNumber("target_dist", targetDist);
    SmartDashboard.putNumber("turret_rel_rotation", getTurretRelRot().getDegrees());
    SmartDashboard.putNumber("turret_abs_rotation", getTurretAbsRot().getDegrees());
    SmartDashboard.putNumber("turret_abs_X", getTurrePosAbsRot().getX());
    SmartDashboard.putNumber("turret_abs_Y", getTurrePosRelRot().getX());
  }
}
