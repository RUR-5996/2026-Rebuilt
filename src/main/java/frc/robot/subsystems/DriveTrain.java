package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.Constants.SwerveConstants;

public class DriveTrain extends SubsystemBase {
    private static DriveTrain instance;

    // Kinematics (Wheel locations relative to robot center)
    public final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(0.3, 0.3),   // FL
        new Translation2d(0.3, -0.3),  // FR
        new Translation2d(-0.3, 0.3),  // BL
        new Translation2d(-0.3, -0.3)  // BR
    );

    private final SwerveModuleDef m_frontLeft;
    private final SwerveModuleDef m_frontRight;
    private final SwerveModuleDef m_backLeft;
    private final SwerveModuleDef m_backRight;

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

    // Helper to set speeds from the SwerveDrive subsystem
    public void setModuleSpeeds(SwerveModuleState[] states) {
        m_frontLeft.setState(states[0]);
        m_frontRight.setState(states[1]);
        m_backLeft.setState(states[2]);
        m_backRight.setState(states[3]);
    }

    // Debugging
    public void spinAllSteerMotors(double rotations) {
        m_frontLeft.forceSteerRotation(rotations);
        m_frontRight.forceSteerRotation(rotations);
        m_backLeft.forceSteerRotation(rotations);
        m_backRight.forceSteerRotation(rotations);
    }
}