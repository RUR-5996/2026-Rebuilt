package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.LimelightConstants;

public class Limelight {

    private static Limelight ROBOT_LIMELIGHT;
    private static Limelight TURRET_LIMELIGHT;

    SwerveDrive SWERVE;
    Shooter SHOOTER;

    private final String limelightName;

    public Limelight(String name) {
        SWERVE = SwerveDrive.getInstance();
        SHOOTER = Shooter.getInstance();

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
        }
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
        LimelightHelpers.SetRobotOrientation(limelightName, SWERVE.getHeading().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate positionEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        boolean spinningTooFastToBeUseful = false;
        if (limelightName == LimelightConstants.ROBOT_LIMELIGHT_NAME) {
            spinningTooFastToBeUseful = Math.abs(SWERVE.getSpinRate()) > LimelightConstants.MAX_VISION_SPIN;
        } else if (limelightName == LimelightConstants.TURRET_LIMELIGHT_NAME) {
            spinningTooFastToBeUseful = (Math.abs(SHOOTER.getTurretVelocity() + SWERVE.getSpinRate())) > LimelightConstants.MAX_VISION_SPIN;
        }

        boolean tagsVisible = positionEstimate.tagCount > 0;
        if (!spinningTooFastToBeUseful && tagsVisible) {
            return positionEstimate.pose;
        } else {
            return new Pose2d(-1.0, -1.0, new Rotation2d(-1.0));
        }
    }
}
