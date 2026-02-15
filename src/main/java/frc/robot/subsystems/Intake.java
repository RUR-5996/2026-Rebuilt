
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{

    private static Intake INTAKE;

    SparkMax intakeFlipOutMotorL;
    RelativeEncoder intakeFlipOutEncoderL;
    SparkClosedLoopController intakeFlipOutControllerL;
    SparkMax intakeFlipOutMotorR;
    RelativeEncoder intakeFlipOutEncoderR;
    SparkClosedLoopController intakeFlipOutControllerR;
    
    SparkMax intakePowerMotor;
    SparkClosedLoopController intakePowerController;

    IntakeState intakeState = IntakeState.IN;
    IntakeSpin intakeSpin = IntakeSpin.OFF;

    public Intake() {
        
        intakeFlipOutMotorL = new SparkMax(IntakeConstants.flipOutMotorLId, MotorType.kBrushless);

        SparkMaxConfig intakeFlipOutConfig = new SparkMaxConfig();
        intakeFlipOutConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        intakeFlipOutConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(1)
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


        intakePowerMotor = new SparkMax (IntakeConstants.powerMotorId, MotorType.kBrushless);

        SparkMaxConfig intakePowerConfig = new SparkMaxConfig();
        intakePowerConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast);
        intakePowerMotor.configure(intakePowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    }

    public static Intake getInstance() {
        if(INTAKE == null) {
            INTAKE = new Intake();
        }
        return INTAKE;
    }

    public void report() {
        SmartDashboard.putNumber("L_pos", intakeFlipOutEncoderL.getPosition());
        SmartDashboard.putNumber("R_pos", intakeFlipOutEncoderR.getPosition());
        SmartDashboard.putString("intakeState", getIntakeState());
    }

    public Command intakeFlipOut() {
        return Commands.runOnce(() -> {
            intakeFlipOutControllerL.setSetpoint(Constants.IntakeConstants.POS_OUT, SparkMax.ControlType.kPosition);
            intakeFlipOutControllerR.setSetpoint(Constants.IntakeConstants.POS_OUT, SparkMax.ControlType.kPosition);
            if (intakeFlipOutEncoderL.getPosition() == Constants.IntakeConstants.POS_OUT) {
                intakeState = IntakeState.OUT;
            }});
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
            intakePowerMotor.set(0.3);
        });
    }

    public Command intakeOff() {
        return Commands.runOnce(() -> {
            intakeSpin = IntakeSpin.OFF;
            intakePowerMotor.stopMotor();
        });
    }

    public String getIntakeState() {
        return intakeState.toString();
    }

    private enum IntakeState {
        IN,
        OUT,
        ERROR,
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
