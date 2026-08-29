package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ExternalEncoderConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.VelocityVoltage;

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
    SoftLimitConfig turretLimitConfig;
    RelativeEncoder turretEncoder;

    private final VelocityVoltage feederVelocityVoltage = new VelocityVoltage(0);

    private double currentShooterSpeed = 0.51;

    public double speedIncrement = 0;
    public double rotationIncrement = 0;

    private double turretXabs = 0.0;
    private double turretYabs = 0.0;
    private double turretRotAbs = 0.0;
    private double turretRotRel = 0.0;

    private double targetDist = 0.0;

    public double x = 0;
    public double y = 0;

    private AimBot aimBot = AimBot.OFF;


    SwerveDrive SWERVE;
    Limelight TURRET_LIMELIGHT;

    public Shooter () {

        SWERVE = SwerveDrive.getInstance();
        TURRET_LIMELIGHT = Limelight.getInstance(LimelightConstants.TURRET_LIMELIGHT_NAME);

        powerMotor1 = new SparkMax(ShooterConstants.POWER_MOTOR_1_ID, MotorType.kBrushless);
        powerMotor2 = new SparkMax(ShooterConstants.POWER_MOTOR_2_ID, MotorType.kBrushless);
        powerConfig = new SparkMaxConfig();

        powerConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(50);
        powerConfig.closedLoop
            .p(ShooterConstants.POWER_MOTOR_P)
            .i(ShooterConstants.POWER_MOTOR_I)
            .d(ShooterConstants.POWER_MOTOR_D)
            .maxOutput(2.0, ClosedLoopSlot.kSlot0);
            //.feedForward.kV(ShooterConstants.POWER_MOTOR_V);
        powerConfig.encoder
            .positionConversionFactor(ShooterConstants.POWER_MOTOR_GEAR_RATIO)
            .velocityConversionFactor(ShooterConstants.POWER_MOTOR_GEAR_RATIO);

        powerConfig.closedLoopRampRate(0.0);

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
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .openLoopRampRate(0.2)
            .closedLoopRampRate(0.2);
        turretConfig.encoder
            .positionConversionFactor(ShooterConstants.MOTOR_TO_TURRET_RATIO);
        turretConfig.closedLoop
            .p(ShooterConstants.TURRET_MOTOR_P)
            .i(ShooterConstants.TURRET_MOTOR_I)
            .d(ShooterConstants.TURRET_MOTOR_D)
            .positionWrappingEnabled(false);
        turretConfig.softLimit
            .forwardSoftLimit(ShooterConstants.MAX_TURRET_ANGLE)
            .reverseSoftLimit(ShooterConstants.MIN_TURRET_ANGLE)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true);


        turretMotor.configure(turretConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        turretController = turretMotor.getClosedLoopController();

        turretEncoder = turretMotor.getEncoder();
        turretEncoder.setPosition(0);
    }

    public static Shooter getInstance() {
        if(SHOOTER == null) {
            SHOOTER = new Shooter();
        }
        return SHOOTER;
    }

    public Command shooterOnDefault() {
            return Commands.runOnce(() -> {
            double targetRPM = ShooterConstants.NEO_MAX_RPM * ShooterConstants.POWER_MOTOR_GEAR_RATIO * ShooterConstants.DEFAULT_SHOOTER_SPEED;
            powerController1.setSetpoint(targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
            powerController2.setSetpoint(targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
            
            });
        }

    public Command shooterOn() {
        return Commands.runOnce(() -> {
            double targetRPM = ShooterConstants.NEO_MAX_RPM * ShooterConstants.POWER_MOTOR_GEAR_RATIO * currentShooterSpeed;
            powerController1.setSetpoint(targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
            powerController2.setSetpoint(targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
            });
        }

    public Command shooterOff() {
        return Commands.runOnce(() -> {
            powerMotor1.stopMotor();
            powerMotor2.stopMotor();
        });  
    }

    public Command feederOn() {
        return //new SequentialCommandGroup(
    //Commands.waitSeconds(0.4),
    Commands.runOnce(() -> {
            feederMotor.setControl(feederVelocityVoltage.withVelocity(ShooterConstants.FEEDER_VELOCITY));    
        });
    }

    public Command feederOff() {
        return Commands.runOnce(() -> {
            feederMotor.stopMotor();
        });
    }


  public void rotateTurret(double targetAngle) {
    turretController.setSetpoint(targetAngle, ControlType.kPosition);
  }


  public Command adjustShooterSpeed(double increment) {
    return Commands.runOnce(() -> {
      speedIncrement += increment;
    });
  }

  public Command adjustShooterRotation(double increment) {
    return Commands.runOnce(() -> {
      rotationIncrement += Math.toRadians(increment);
    });
  }


  public void calcShooterSpeed() {
    currentShooterSpeed = ShooterConstants.DEFAULT_SHOOTER_SPEED + speedIncrement;
  }


  public void calcTurretXY() {   //position and rotation of robot

    Pose2d pos = SWERVE.getPose();
    double phi = ShooterConstants.VEC_TURRET_PHI + clampRot(pos.getRotation().getRadians());
    turretXabs = ShooterConstants.VEC_TURRET_LEN*Math.cos(phi)+pos.getX();
    turretYabs = ShooterConstants.VEC_TURRET_LEN*Math.sin(phi)+pos.getY();
  }


  public void calcTurretAbsRotation() {
    
    double deltaX = x - turretXabs;
    double deltaY = y - turretYabs;

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
    Rotation2d robotRot = SWERVE.getPose().getRotation();
    turretRotRel = clampRot(turretRotAbs - clampRot(robotRot.getRadians()) + Math.toRadians(-90)) + rotationIncrement;
  }

  public double clampRot(double rot) {   //internal calculations are in radians
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


  public void rotateStop() {
    turretMotor.stopMotor();
  }

  public Command rotateStopCommand() {
    return Commands.runOnce(() -> {
      turretMotor.stopMotor();
    });
  }

  public Command shootOut () {
    return new SequentialCommandGroup(feederOn(), shooterOn(), new WaitCommand(5.0), feederOff(), shooterOff());
  }

  public Command aimOn () {
    return Commands.runOnce(() -> {
      aimBot = AimBot.ON;
    });
  }

    public Command aimOff () {
    return Commands.runOnce(() -> {
      aimBot = AimBot.OFF;
    });
  }


  public BooleanSupplier inRange() {
    return () -> (targetDist > ShooterConstants.MINIMUM_SHOOTING_DISTANCE);
  }

  public SequentialCommandGroup shootAndFeed() {
    return new SequentialCommandGroup(shooterOn(), 
      new WaitUntilCommand(() -> Math.min(powerEncoder1.getVelocity(), powerEncoder2.getVelocity()) 
        >= (ShooterConstants.NEO_MAX_RPM * ShooterConstants.POWER_MOTOR_GEAR_RATIO * currentShooterSpeed)),
      feederOn()
      );
  }

  public void periodic() {
    calcTurretXY();
    calcTurretAbsRotation();
    calcTurretRelRotation();
    calcShooterSpeed();
    if (aimBot == AimBot.ON) {
      rotateTurret(Math.toDegrees(turretRotRel));
      
    } else {
      rotateStop();
    }

  }

  private enum AimBot {
    ON(true),
    OFF(false);

    public boolean val;

    private AimBot(boolean val) {
      this.val = val;
    }
  }

  }
