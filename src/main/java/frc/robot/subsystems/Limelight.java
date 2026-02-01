package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.LimelightConstants;

public class Limelight {

    private static Limelight LIMELIGHT;
    SwerveDrive SWERVE;

    public Limelight() {
        SWERVE = SwerveDrive.getInstance();

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

    public static Limelight getInstance() {
        if(LIMELIGHT == null) {
            LIMELIGHT = new Limelight();
        }
        return LIMELIGHT;
    }

    public Pose2d apriltagBasedPosition() {
        LimelightHelpers.SetRobotOrientation("limelight", SWERVE.getHeading().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate positionEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        boolean spinningTooFastToBeUseful = Math.abs(SWERVE.getSpinRate()) > 720;
        boolean tagsVisible = positionEstimate.tagCount > 0;
        if (!spinningTooFastToBeUseful && tagsVisible) {
            return positionEstimate.pose;
        } else {
            return new Pose2d(-1.0, -1.0, new Rotation2d(-1.0));
        }
    }
}
