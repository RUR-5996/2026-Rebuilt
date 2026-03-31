package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Util.LimelightHelpers;

public class Limelight extends SubsystemBase{

    private static Limelight ROBOT_LIMELIGHT;
    private static Limelight TURRET_LIMELIGHT;

    SwerveDrive SWERVE;
    Shooter SHOOTER;

    private final String limelightName;

    public Limelight(String name) {
        SWERVE = SwerveDrive.getInstance();

        limelightName = name;

        if (limelightName == LimelightConstants.ROBOT_LIMELIGHT_NAME) {
            LimelightHelpers.setCameraPose_RobotSpace(
                LimelightConstants.ROBOT_LIMELIGHT_NAME,
                LimelightConstants.ROBOT_LIMELIGHT_OFFSET.getX(),
                LimelightConstants.ROBOT_LIMELIGHT_OFFSET.getY(),
                LimelightConstants.ROBOT_LIMELIGHT_OFFSET.getZ(),
                Math.toDegrees(LimelightConstants.ROBOT_LIMELIGHT_OFFSET.getRotation().getX()), // roll
                Math.toDegrees(LimelightConstants.ROBOT_LIMELIGHT_OFFSET.getRotation().getY()), // pitch
                Math.toDegrees(LimelightConstants.ROBOT_LIMELIGHT_OFFSET.getRotation().getZ())  // yaw
            );
        } else if (limelightName == LimelightConstants.TURRET_LIMELIGHT_NAME) {
            LimelightHelpers.setCameraPose_RobotSpace(
                LimelightConstants.TURRET_LIMELIGHT_NAME,
                LimelightConstants.TURRET_LIMELIGHT_OFFSET.getX(),
                LimelightConstants.TURRET_LIMELIGHT_OFFSET.getY(),
                LimelightConstants.TURRET_LIMELIGHT_OFFSET.getZ(),
                Math.toDegrees(LimelightConstants.TURRET_LIMELIGHT_OFFSET.getRotation().getX()), // roll
                Math.toDegrees(LimelightConstants.TURRET_LIMELIGHT_OFFSET.getRotation().getY()), // pitch
                Math.toDegrees(LimelightConstants.TURRET_LIMELIGHT_OFFSET.getRotation().getZ())  // yaw
            );
    }
}

    public void report() {
        Pose2d estimated_position = apriltagBasedPosition();
        SmartDashboard.putNumber("x_estimation", estimated_position.getX());
        SmartDashboard.putNumber("y_estimation", estimated_position.getY());
        SmartDashboard.putNumber("rot_estimation",  estimated_position.getRotation().getDegrees());
        //SmartDashboard.putNumber("visible_apriltags", getTagCount());
    }

    public static Limelight getInstance(String name) {
        if (name == LimelightConstants.ROBOT_LIMELIGHT_NAME) {
            if (ROBOT_LIMELIGHT == null) {
                ROBOT_LIMELIGHT = new Limelight(LimelightConstants.ROBOT_LIMELIGHT_NAME);
            }
            return ROBOT_LIMELIGHT;
        } else {
            if (TURRET_LIMELIGHT == null) {
                TURRET_LIMELIGHT = new Limelight(LimelightConstants.TURRET_LIMELIGHT_NAME);
            }
            return TURRET_LIMELIGHT;
        }
    }

    public Pose2d apriltagBasedPosition() {
        LimelightHelpers.SetRobotOrientation(limelightName, SWERVE.gyro.getAngle(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate positionEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        //return positionEstimate.pose;

        boolean spinningTooFastToBeUseful = false;
        if (limelightName == LimelightConstants.ROBOT_LIMELIGHT_NAME) {
            spinningTooFastToBeUseful = Math.abs(SWERVE.gyro.getRate()) > LimelightConstants.MAX_VISION_SPIN;
        } /*else if (limelightName == LimelightConstants.TURRET_LIMELIGHT_NAME) {
            spinningTooFastToBeUseful = (Math.abs(SHOOTER.getTurretVelocity() + SWERVE.getSpinRate())) > LimelightConstants.MAX_VISION_SPIN;
        }*/
        boolean tagsVisible = false;
        if (positionEstimate != null) {
            tagsVisible = positionEstimate.tagCount > 0;
        } else{
            
            return new Pose2d(-1.0, -1.0, new Rotation2d(-1.0));
        }
        
        if (!spinningTooFastToBeUseful && tagsVisible) {
            return positionEstimate.pose;
        } else {
            return new Pose2d(-1.0, -1.0, new Rotation2d(-1.0)); //TODO return Null and keep the original pose
        }
    }

    public int getTagCount() {
        LimelightHelpers.SetRobotOrientation(limelightName, SWERVE.gyro.getAngle(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate positionEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        
        return positionEstimate.tagCount;

    }

    public Double getTurretAngle() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(LimelightConstants.TURRET_LIMELIGHT_NAME);
            double tv = table.getEntry("tv").getDouble(0.0); // 0 = no target, 1 = target
            if (tv < 1.0) {
                return null;
            }
        return table.getEntry("tx").getDouble(0.0); // horizontal offset in degrees
    }

    public Command updateRobotPosition() {
        return Commands.runOnce(() -> {
            Pose2d pose = apriltagBasedPosition();
            SWERVE.resetOdometry(new Pose2d(pose.getX(), pose.getY(), new Rotation2d(Math.toRadians(SWERVE.gyro.getAngle()))));
        });
    }
}