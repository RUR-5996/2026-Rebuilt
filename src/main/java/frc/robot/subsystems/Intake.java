
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Intake extends SubsystemBase{

    private static Intake INTAKE;

    SparkMax intakeFlipOutMotorL;
    RelativeEncoder intakeFlipOutEncoderL;
    SparkClosedLoopController intakeFlipOutControllerL;
    SparkMax intakeFlipOutMotorR;
    RelativeEncoder intakeFlipOutEncoderR;
    SparkClosedLoopController intakeFlipOutControllerR;
    
    //SparkMax intakePowerMotor;
    TalonFX intakePowerMotor;
    TalonFXConfiguration intakePowerConfig;
    //SparkClosedLoopController intakePowerController;

    IntakeState intakeState = IntakeState.IN;
    IntakeSpin intakeSpin = IntakeSpin.OFF;
    

    private double setPoint = 0;

    public Intake() {
        
        intakeFlipOutMotorL = new SparkMax(IntakeConstants.flipOutMotorLId, MotorType.kBrushless);

        SparkMaxConfig intakeFlipOutConfig = new SparkMaxConfig();
        intakeFlipOutConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        intakeFlipOutConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(1) //TODO make motors turn the same angle
            .i(0.0)
            .d(0.0)
            .outputRange(-0.15, 0.15)
            .positionWrappingEnabled(false);
        intakeFlipOutConfig.encoder.positionConversionFactor(Constants.IntakeConstants.FLIPOUT_COEFFICIENT);

        intakeFlipOutMotorL.configure(intakeFlipOutConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        intakeFlipOutEncoderL = intakeFlipOutMotorL.getEncoder();
        intakeFlipOutControllerL = intakeFlipOutMotorL.getClosedLoopController();
        intakeFlipOutEncoderL.setPosition(0);


        intakeFlipOutMotorR = new SparkMax(IntakeConstants.flipOutMotorRId, MotorType.kBrushless);

        intakeFlipOutConfig
            .inverted(true);
        
        intakeFlipOutMotorR.configure(intakeFlipOutConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        intakeFlipOutEncoderR = intakeFlipOutMotorR.getEncoder();
        intakeFlipOutControllerR = intakeFlipOutMotorR.getClosedLoopController();
        intakeFlipOutEncoderR.setPosition(0);

        //intakePowerMotor = new SparkMax (IntakeConstants.powerMotorId, MotorType.kBrushless);
        intakePowerMotor = new TalonFX(IntakeConstants.powerMotorId);
        intakePowerConfig = new TalonFXConfiguration();
        intakePowerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakePowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakePowerConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.2;
        intakePowerConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;

        intakePowerMotor.getConfigurator().apply(intakePowerConfig);

        /*SparkMaxConfig intakePowerConfig = new SparkMaxConfig();
        intakePowerConfig
            .inverted(true)
            .idleMode(IdleMode.kCoast);
        intakePowerMotor.configure(intakePowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        */
    }

    public static Intake getInstance() {
        if(INTAKE == null) {
            INTAKE = new Intake();
        }
        return INTAKE;
    }

    public void brakeIntake() {
        SparkMaxConfig intakeFlipOutConfig = new SparkMaxConfig();
        intakeFlipOutConfig.idleMode(IdleMode.kBrake);
        intakeFlipOutMotorL.configure(intakeFlipOutConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        intakeFlipOutMotorR.configure(intakeFlipOutConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void coastIntake() {
        SparkMaxConfig intakeFlipOutConfig = new SparkMaxConfig();
        intakeFlipOutConfig.idleMode(IdleMode.kCoast);
        intakeFlipOutMotorL.configure(intakeFlipOutConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        intakeFlipOutMotorR.configure(intakeFlipOutConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void report() {
        SmartDashboard.putNumber("L_pos", intakeFlipOutEncoderL.getPosition());
        SmartDashboard.putNumber("R_pos", intakeFlipOutEncoderR.getPosition());
        SmartDashboard.putNumber("setPoint", setPoint);
        SmartDashboard.putString("intakeState", getIntakeState());
        SmartDashboard.putBoolean("Is intake out?", intakeState.val);
    }

    public Command intakeFlipOut() {
        return new SequentialCommandGroup(Commands.runOnce(() -> {
            intakeFlipOutControllerL.setSetpoint(Constants.IntakeConstants.POS_OUT, SparkMax.ControlType.kPosition);
            intakeFlipOutControllerR.setSetpoint(Constants.IntakeConstants.POS_OUT, SparkMax.ControlType.kPosition);
            if (intakeFlipOutEncoderL.getPosition() == Constants.IntakeConstants.POS_OUT) {
                intakeState = IntakeState.OUT;
            }}),
            Commands.waitSeconds(1.5),
            Commands.runOnce(() -> {
                coastIntake();
                intakeFlipOutMotorL.stopMotor();
                intakeFlipOutMotorR.stopMotor();
            }));
        /*return Commands.runOnce(() -> {
        if (intakeState == IntakeState.IN){
            intakeState = IntakeState.OUT;
            moveByRotations(2, intakeFlipOutEncoderL, intakeFlipOutControllerL);
            moveByRotations(2, intakeFlipOutEncoderR, intakeFlipOutControllerR);
            }
        });*/
    }

    public Command intakeFlipIn() {
        return Commands.runOnce(() -> {
            brakeIntake();
            intakeFlipOutControllerL.setSetpoint(Constants.IntakeConstants.POS_IN, SparkMax.ControlType.kPosition);
            intakeFlipOutControllerR.setSetpoint(Constants.IntakeConstants.POS_IN, SparkMax.ControlType.kPosition);
            if (intakeFlipOutEncoderL.getPosition() == Constants.IntakeConstants.POS_IN) {
                intakeState = IntakeState.IN;
        }});
        /*return Commands.runOnce(() -> {
        if (intakeState == IntakeState.IN){
            intakeState = IntakeState.OUT;
            moveByRotations(-2, intakeFlipOutEncoderL, intakeFlipOutControllerL);
            moveByRotations(-2, intakeFlipOutEncoderR, intakeFlipOutControllerR);
            }
        });*/
    }

    public Command intakeOn() {
        return Commands.runOnce(() -> {
            intakeSpin = IntakeSpin.ON;
            intakePowerMotor.set(Constants.IntakeConstants.SPEED);
        });
    }

    public Command intakeOff() {
        return Commands.runOnce(() -> {
            intakeSpin = IntakeSpin.OFF;
            intakePowerMotor.stopMotor();
        });
    }

    public Command nudgeUp() {
        return Commands.runOnce(() -> {
            setPoint -= .01;
            intakeFlipOutControllerL.setSetpoint(setPoint, ControlType.kPosition);
            intakeFlipOutControllerR.setSetpoint(setPoint, ControlType.kPosition);
        });
    }

    public Command nudgeDown() {
        return Commands.runOnce(() -> {
            setPoint += .01;
            intakeFlipOutControllerL.setSetpoint(setPoint, ControlType.kPosition);
            intakeFlipOutControllerR.setSetpoint(setPoint, ControlType.kPosition);
        });
    }

    public String getIntakeState() {
        return intakeState.toString();
    }

    private enum IntakeState {
        IN(false),
        OUT(true),
        ERROR(false);

        public boolean val;

        private IntakeState(boolean val) {
            this.val = val;
        }
    }

    private enum IntakeSpin {
        ON,
        OFF,
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
