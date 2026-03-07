
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
import com.ctre.phoenix6.configs.CANcoderConfiguration;
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

    CANcoder turretCANcoder;
    CANcoderConfiguration turretCANcoderConfig;

    private final VelocityVoltage feederVelocityVoltage = new VelocityVoltage(0);

    private double currentShooterSpeed = 0.5;

    private double turretXabs = 0.0;
    private double turretYabs = 0.0;
    private double turretRotAbs = 0.0;
    private double turretRotRel = 0.0;

    private double targetX = 4.5;
    private double targetY = 4.0;
    private double targetDist = 0.0;

    private AimBot aimBot = AimBot.OFF;

    public SwerveDrive SWERVE;

    public Shooter () {

        SWERVE = SwerveDrive.getInstance();

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
            .positionConversionFactor(ShooterConstants.POWER_MOTOR_GEAR_RATIO)
            .velocityConversionFactor(ShooterConstants.POWER_MOTOR_GEAR_RATIO);

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
            //.inverted(true)
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

        //turretConfig.apply(turretLiConfig);

        turretMotor.configure(turretConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        turretController = turretMotor.getClosedLoopController();

        turretEncoder = turretMotor.getEncoder();
        turretEncoder.setPosition(0);

        /*
        turretCANcoder = new CANcoder(ShooterConstants.TURRET_CANCODER_ID);
        turretCANcoderConfig = new CANcoderConfiguration();

        turretCANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        turretCANcoder.getConfigurator().apply(turretCANcoderConfig);
*/   
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

    public Command shooterOn(double speedPercentage) {
            return Commands.runOnce(() -> {
            double targetRPM = ShooterConstants.NEO_MAX_RPM * ShooterConstants.POWER_MOTOR_GEAR_RATIO * speedPercentage;
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
            //powerMotor1.set(0.0);
            //powerMotor2.set(0.0);
            powerMotor1.stopMotor();
            powerMotor2.stopMotor();
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

  /*public Command rotateTurret (double targetAngle) { //everything is in degrees   TODO make conversion do radians
    return Commands.runOnce(() -> {
        turretController.setSetpoint(targetAngle, ControlType.kPosition);
    });
  }*/

  public void rotateTurret(double targetAngle) {
    turretController.setSetpoint(targetAngle, ControlType.kPosition);
  }


  public Command adjustShooterSpeed(boolean faster) {
    return Commands.runOnce(() -> {
        if (faster) {
          currentShooterSpeed += 0.02;
        } else {
          currentShooterSpeed -= 0.02;
        }
    });
  }

  public void calcShooterSpeed() {
    double speed = - 0.0362 * Math.pow(targetDist, 4) + 0.6903 * Math.pow(targetDist, 3) - 4.8608 * Math.pow(targetDist, 2) + 15.046 * targetDist - 16.858;
    if (speed <= 0.3) {
      speed = 0.3;
    }
    currentShooterSpeed = speed;
  }


  public void calcTurretXY() {   //position and rotation of robot
    Pose2d pos = SWERVE.getPose();
    double phi = ShooterConstants.VEC_TURRET_PHI + clampRot(pos.getRotation().getRadians());
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
    Rotation2d robotRot = SWERVE.getPose().getRotation();
    turretRotRel = clampRot(turretRotAbs - clampRot(robotRot.getRadians()));
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

  public boolean aimBotOn() {
    if (aimBot == AimBot.ON) {
      return true;
    } else {
      return false;
    }
  }


  public void periodic() {
    calcTurretXY();
    calcTurretAbsRotation();
    calcTurretRelRotation();
    if (aimBot == AimBot.ON) {
      rotateTurret(Math.toDegrees(turretRotRel));
      calcShooterSpeed();
    } else {
      rotateStop();
    }
  }

  public void report() {
    SmartDashboard.putNumber("target_dist", targetDist);
    SmartDashboard.putNumber("turret_rel_rotation", getTurretRelRot().getDegrees());
    SmartDashboard.putNumber("turret_abs_rotation", getTurretAbsRot().getDegrees());
    SmartDashboard.putNumber("turret_abs_X", getTurrePosAbsRot().getX());
    SmartDashboard.putNumber("turret_abs_Y", getTurrePosRelRot().getX());
    SmartDashboard.putNumber("current shooting speed", powerEncoder1.getVelocity());
    SmartDashboard.putNumber("requested shooting speed", powerController1.getSetpoint());
    SmartDashboard.putNumber("calculated relative turret heading", Math.toDegrees(turretRotRel));
    SmartDashboard.putNumber("current shooter speed", currentShooterSpeed);
    SmartDashboard.putNumber("current turret angle", turretEncoder.getPosition());
    SmartDashboard.putBoolean("aimbot", aimBotOn());
    SmartDashboard.putBoolean("can shoot?", targetDist > ShooterConstants.MINIMUM_SHOOTING_DISTANCE);
  }

  private enum AimBot {
    ON,
    OFF
  }

}


