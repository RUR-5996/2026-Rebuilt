package frc.robot.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.SwerveDrive;

public class FieldReport {
    
     Field2d field;

    static SwerveDrive SWERVE;
    PowerDistribution PDP;
    static DriveTrain DRIVETRAIN;
    static FieldReport REPORT;

    public FieldReport() {
        SWERVE = SwerveDrive.getInstance();
        DRIVETRAIN = DriveTrain.getInstance();
        PDP = new PowerDistribution(0, ModuleType.kCTRE);

        setupField();
    }

    public void setupField() {
        field = new Field2d();
        SmartDashboard.putData("field", field);
        field.setRobotPose(SWERVE.getPose());
        //reportSwerve();
    }

    public void periodic() {
        field.setRobotPose(SWERVE.getPose());
    }

    public static FieldReport getInstance() {
        if(REPORT == null) {
            REPORT = new FieldReport();
        }
        return REPORT;
    }

    public void reportSwerve() {
        SmartDashboard.putData("Swerve Drive", new Sendable() { //all headings are in degrees
            @Override
                public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", () -> DRIVETRAIN.m_frontLeft.getSteerAngle(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> DRIVETRAIN.m_frontLeft.getSpeed(), null);

                builder.addDoubleProperty("Front Right Angle", () -> DRIVETRAIN.m_frontRight.getSteerAngle(), null);
                builder.addDoubleProperty("Front Right Velocity", () -> DRIVETRAIN.m_frontRight.getSpeed(), null);

                builder.addDoubleProperty("Back Left Angle", () -> DRIVETRAIN.m_backLeft.getSteerAngle(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> DRIVETRAIN.m_backLeft.getSpeed(), null);

                builder.addDoubleProperty("Back Right Angle", () -> DRIVETRAIN.m_backRight.getSteerAngle(), null);
                builder.addDoubleProperty("Back Right Velocity", () -> DRIVETRAIN.m_backLeft.getSpeed(), null);

                builder.addDoubleProperty("Robot Angle", () -> SWERVE.getPose().getRotation().getDegrees(), null);
            }
        });
    }
}

