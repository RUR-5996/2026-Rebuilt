package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.SwerveConstants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveTrain extends SubsystemBase {
    private static DriveTrain instance;

    // Kinematics (Wheel locations relative to robot center)
    public final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(0.26, 0.26),   // FL
        new Translation2d(0.26, -0.26),  // FR
        new Translation2d(-0.26, 0.26),  // BL
        new Translation2d(-0.26, -0.26)  // BR
    );

    public final SwerveModuleDef m_frontLeft;
    public final SwerveModuleDef m_frontRight;
    public final SwerveModuleDef m_backLeft;
    public final SwerveModuleDef m_backRight;

    private DriveTrain() {
        m_frontLeft = new SwerveModuleDef(new TalonFX(2), new TalonFX(1), SwerveConstants.FL_STEER_INVERT, InvertedValue.CounterClockwise_Positive);
        m_frontRight = new SwerveModuleDef(new TalonFX(4), new TalonFX(3), SwerveConstants.FR_STEER_INVERT, InvertedValue.Clockwise_Positive);
        m_backLeft = new SwerveModuleDef(new TalonFX(6), new TalonFX(5), SwerveConstants.BL_STEER_INVERT, InvertedValue.CounterClockwise_Positive);
        m_backRight = new SwerveModuleDef(new TalonFX(8), new TalonFX(7), SwerveConstants.BR_STEER_INVERT, InvertedValue.Clockwise_Positive);
    }

    public static DriveTrain getInstance() {
        if (instance == null) instance = new DriveTrain();
        return instance;
    }

    // Helper to get all module positions for Odometry
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            m_frontLeft.getModulePosition(),
            m_frontRight.getModulePosition(),
            m_backLeft.getModulePosition(),
            m_backRight.getModulePosition()
        };
    }

    public ChassisSpeeds getSpeeds() {
        return swerveKinematics.toChassisSpeeds(new SwerveModuleState[]{m_frontLeft.getModuleState(), m_frontRight.getModuleState(), m_backLeft.getModuleState(), m_backRight.getModuleState()});
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            m_frontLeft.getModuleState(),
            m_frontRight.getModuleState(),
            m_backLeft.getModuleState(),
            m_backRight.getModuleState()
        };
    }

    // Helper to set speeds from the SwerveDrive subsystem
    // Helper to set speeds from the SwerveDrive subsystem
    public void setModuleSpeeds(SwerveModuleState[] states) {
        m_frontLeft.setState(states[0]);
        m_frontRight.setState(states[1]);
        m_backLeft.setState(states[2]);
        m_backRight.setState(states[3]);
    }

    public void setToCoast() {
        m_frontLeft.setDriveNeutralMode(NeutralModeValue.Coast);
        m_frontRight.setDriveNeutralMode(NeutralModeValue.Coast);
        m_backLeft.setDriveNeutralMode(NeutralModeValue.Coast);
        m_backRight.setDriveNeutralMode(NeutralModeValue.Coast);

        m_frontLeft.setSteerNeutralMode(NeutralModeValue.Coast);
        m_frontRight.setSteerNeutralMode(NeutralModeValue.Coast);
        m_backLeft.setSteerNeutralMode(NeutralModeValue.Coast);
        m_backRight.setSteerNeutralMode(NeutralModeValue.Coast);
    }

    
    public void setToBrake() {
        m_frontLeft.setDriveNeutralMode(NeutralModeValue.Brake);
        m_frontRight.setDriveNeutralMode(NeutralModeValue.Brake);
        m_backLeft.setDriveNeutralMode(NeutralModeValue.Brake);
        m_backRight.setDriveNeutralMode(NeutralModeValue.Brake); 
        
        m_frontLeft.setSteerNeutralMode(NeutralModeValue.Brake);
        m_frontRight.setSteerNeutralMode(NeutralModeValue.Brake);
        m_backLeft.setSteerNeutralMode(NeutralModeValue.Brake);
        m_backRight.setSteerNeutralMode(NeutralModeValue.Brake);
    }
    
    public Command resetSteer() {
        return Commands.runOnce(() -> {
            m_frontLeft.resetSteerEncoder();
            m_frontRight.resetSteerEncoder();
            m_backLeft.resetSteerEncoder();
            m_backRight.resetSteerEncoder();
        });     
    }
    
    // Debugging
    public void spinAllSteerMotors(double rotations) {
        m_frontLeft.forceSteerRotation(rotations);
        m_frontRight.forceSteerRotation(rotations);
        m_backLeft.forceSteerRotation(rotations);
        m_backRight.forceSteerRotation(rotations);
    }
}