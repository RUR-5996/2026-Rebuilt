package frc.robot.util;


import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.SwerveDrive;


public class Report {
    Field2d field;

    static SwerveDrive SWERVE;
    PowerDistribution PDP;
    static DriveTrain DRIVETRAIN;
    static Report REPORT;

    public Report() {
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

    public static Report getInstance() {
        if(REPORT == null) {
            REPORT = new Report();
        }
        return REPORT;
    }

    public static void reportSwerve() {
        SwerveModuleState[] moduleStates = DRIVETRAIN.getModuleStates();

        SmartDashboard.putData("Swerve Drive", new Sendable() { //all headings are in degrees
            @Override
                public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", () -> moduleStates[0].angle.getDegrees(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> moduleStates[0].speedMetersPerSecond, null);

                builder.addDoubleProperty("Front Right Angle", () -> moduleStates[1].angle.getDegrees(), null);
                builder.addDoubleProperty("Front Right Velocity", () -> moduleStates[1].speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Left Angle", () -> moduleStates[2].angle.getDegrees(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> moduleStates[2].speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Right Angle", () -> moduleStates[3].angle.getDegrees(), null);
                builder.addDoubleProperty("Back Right Velocity", () -> moduleStates[3].speedMetersPerSecond, null);

                builder.addDoubleProperty("Robot Angle", () -> SWERVE.getAngleDegrees(), null);
            }
        });
    }
}