package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Util.LimelightHelpers;

public class SwerveDrive extends SubsystemBase {

   public final DriveTrain DRIVETRAIN;
   private static SwerveDrive SWERVE;

   private boolean slowmode = false;
   private boolean holdAngleEnabled = false;
   private double holdAngle = 0;

   private boolean fieldRelative = true;

   private final SwerveDrivePoseEstimator m_odometry;
   public final AHRS gyro;

   Pose2d robotPose = new Pose2d();
   SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
   ChassisSpeeds chassisSpeeds;

   private final PIDController rotationController;

   public SwerveDrive() {
      DRIVETRAIN = DriveTrain.getInstance();

      gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

      // The 2025/2026 Pose Estimator constructor
      m_odometry = new SwerveDrivePoseEstimator(
            DRIVETRAIN.swerveKinematics, // 1. Kinematics
            new Rotation2d(gyro.getRotation2d().getRadians()), // 2. Gyro Angle
            DRIVETRAIN.getModulePositions(), // 3. Module Positions
            new Pose2d() // 4. Initial Pose
      );

      setFieldOriented();

      rotationController = new PIDController(
            SwerveConstants.steerKP,
            SwerveConstants.steerKI,
            SwerveConstants.steerKD);

      rotationController.enableContinuousInput(-180, 180);
      rotationController.setTolerance(2);

      chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 0, new Rotation2d(gyro.getRotation2d().getRadians()));
   }

   @Override
   public void periodic() {
      // Update odometry with current gyro heading and module positions (from Phoenix
      // 6)
      m_odometry.update(new Rotation2d(Math.toRadians(gyro.getAngle())), DRIVETRAIN.getModulePositions());
      // updateOdometry();

      SmartDashboard.putNumber("Robot X", getPose().getX());
      SmartDashboard.putNumber("Robot Y", getPose().getY());
      SmartDashboard.putNumber("Robot Gyro Angle", gyro.getRotation2d().getDegrees());
   }

   public static SwerveDrive getInstance() {
      if (SWERVE == null) {
         SWERVE = new SwerveDrive();
      }
      return SWERVE;
   }

   /**
    * Main drive command for teleop.
    */
   public Command joystickDrive(DoubleSupplier lx, DoubleSupplier ly, DoubleSupplier rx) {
      return Commands.run(() -> {

         boolean isRedAlliance = isRedAlliance();

         // Apply deadband and scaling
         double xSpeed = (isRedAlliance ? -1 : 1) * MathUtil.applyDeadband(lx.getAsDouble(), 0.1)
               * SwerveConstants.MAX_SPEED_METERS_PER_SECOND;
         double ySpeed = (isRedAlliance ? -1 : 1) * MathUtil.applyDeadband(ly.getAsDouble(), 0.1)
               * SwerveConstants.MAX_SPEED_METERS_PER_SECOND;
         double rot;

         if (holdAngleEnabled) {
            rot = rotationController.calculate(gyro.getAngle(), holdAngle);
         } else {
            rot = MathUtil.applyDeadband(rx.getAsDouble(), 0.1) * SwerveConstants.MAX_ANGULAR_SPEED;
         }

         if (slowmode) {
            xSpeed *= DriverConstants.PRECISION_RATIO;
            ySpeed *= DriverConstants.PRECISION_RATIO;
            rot *= DriverConstants.PRECISION_RATIO;
         }

         // Create ChassisSpeeds (Field Relative)
         ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
               // xSpeed, ySpeed, rot, getHeading()
               xSpeed, ySpeed, rot, new Rotation2d(Math.toRadians(gyro.getAngle())));

         // Convert to module states and desaturate
         SwerveModuleState[] states = DRIVETRAIN.swerveKinematics.toSwerveModuleStates(speeds);
         SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_SPEED_METERS_PER_SECOND);

         DRIVETRAIN.setModuleSpeeds(states);
      }, this);
   }

   // --- Helpers ---

   public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
      ChassisSpeeds speeds;

      if (fieldRelative) {
         // speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
         // getHeading());
         speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
               new Rotation2d(Math.toRadians(gyro.getAngle())));
      } else {
         speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
      }

      SwerveModuleState[] states = DRIVETRAIN.swerveKinematics.toSwerveModuleStates(speeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_SPEED_METERS_PER_SECOND);
      DRIVETRAIN.setModuleSpeeds(states);
   }

   // DO NOT USE FOR ROBOT DRIVING!
   /*
    * public Rotation2d getHeading() {
    * return gyro.getRotation2d();
    * }
    */
   public Pose2d getPose() {
      return m_odometry.getEstimatedPosition();
   }

   public void resetOdometry(Pose2d pose) {
      m_odometry.resetPosition(new Rotation2d(Math.toRadians(gyro.getAngle())), DRIVETRAIN.getModulePositions(), pose);
   }

   public double getOdometryDegrees() {
      return getPose().getRotation().getDegrees();
   }

   public DoubleSupplier supplyOdometryDegrees() {
      DoubleSupplier angle = () -> getOdometryDegrees();
      return angle;
   }

   public double getyMeters() {
      return m_odometry.getEstimatedPosition().getY();
   }

   public double getxMeters() {
      return m_odometry.getEstimatedPosition().getX();
   }

   public double getHoldAngle() {
      return holdAngle;
   }

   public ChassisSpeeds getChassisSpeeds() {
      return chassisSpeeds;
   }

   public Pose2d getOdometryPose() {
      return m_odometry.getEstimatedPosition();
   }

   public SwerveDriveKinematics getKinematics() {
      return DRIVETRAIN.swerveKinematics;
   }

   public boolean getHoldAngleEnabled() {
      return holdAngleEnabled;
   }

   public ChassisSpeeds getActualSpeeds() {
      return DRIVETRAIN.getSpeeds();
   }

   public void setFieldOriented() {
      fieldRelative = true;
      holdAngle = Math.toRadians(gyro.getAngle());
   }

   public void setHoldAngle(double angle) { // in DEGREES
      holdAngle = angle;
   }

   public void setHoldAngleFlag(boolean flag) {
      holdAngleEnabled = flag;
   }

   public void setAutoModuleStates(SwerveModuleState[] states) {
      DRIVETRAIN.setModuleSpeeds(states);
   }

   public void setAutoChassisSpeeds(ChassisSpeeds speeds) {
      chassisSpeeds = speeds;
      setAutoModuleStates(getKinematics().toSwerveModuleStates(speeds));
   }

   public void startMatchPos() {
      Rotation2d matchStartRotation = isRedAlliance() ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);

      gyro.reset();
      gyro.setAngleAdjustment(matchStartRotation.getDegrees());

      // Resetting gyro usually requires resetting odometry to keep them synced
      resetOdometry(new Pose2d(getPose().getTranslation(), matchStartRotation));
   }

   public Command resetGyro() {
      return Commands.runOnce(() -> {
         Rotation2d matchStartRotation = isRedAlliance() ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);

         gyro.reset();
         gyro.setAngleAdjustment(matchStartRotation.getDegrees());

         // Resetting gyro usually requires resetting odometry to keep them synced
         resetOdometry(new Pose2d(getPose().getTranslation(), matchStartRotation));
      });
   }

   public void setSlowmode(boolean enabled) {
      this.slowmode = enabled;
   }

   public boolean isRedAlliance() {
      var alliance = DriverStation.getAlliance();
      return alliance.isPresent() && alliance.get() == Alliance.Red;
   }

   public void updateOdometry() {
      m_odometry.update(
            gyro.getRotation2d(),
            DRIVETRAIN.getModulePositions()
      );

      boolean useMegaTag2 = true; // set to false to use MegaTag1
      boolean doRejectUpdate = false;
      if (!useMegaTag2) {
         LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-robot");

         if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
            if (mt1.rawFiducials[0].ambiguity > .7) {
               doRejectUpdate = true;
            }
            if (mt1.rawFiducials[0].distToCamera > 3) {
               doRejectUpdate = true;
            }
         }
         if (mt1.tagCount == 0) {
            doRejectUpdate = true;
         }

         if (!doRejectUpdate) {
            m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
            m_odometry.addVisionMeasurement(
                  mt1.pose,
                  mt1.timestampSeconds);
         }
      } else if (useMegaTag2) {
         LimelightHelpers.SetRobotOrientation(LimelightConstants.ROBOT_LIMELIGHT_NAME,
               m_odometry.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
         LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.ROBOT_LIMELIGHT_NAME);
         if (Math.abs(gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore
                                               // vision updates
         {
            doRejectUpdate = true;
         }
         if (mt2.tagCount == 0) {
            doRejectUpdate = true;
         }
         if (!doRejectUpdate) {
            m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            m_odometry.addVisionMeasurement(
                  mt2.pose,
                  mt2.timestampSeconds);
         }
      }
   }

   // Debugging
   public Command driveWheelSpins(double spins) {
      // Calculate distance: 3 rotations * Circumference
      double wheelCircumference = 2 * Math.PI * SwerveConstants.WHEEL_RADIUS_METERS;
      double targetDistanceMeters = spins * wheelCircumference;

      return Commands.runOnce(() -> {
         // 1. Reset Pose to 0,0,0 so we can measure distance easily
         // (Only do this if you are testing in isolation, otherwise capture starting
         // pose)
         resetOdometry(new Pose2d());
      }, this)
            .andThen(
                  // 2. Drive forward at 1.0 m/s
                  Commands.run(() -> {
                     ChassisSpeeds speeds = new ChassisSpeeds(1.0, 0, 0);

                     // Convert to module states
                     SwerveModuleState[] states = DRIVETRAIN.swerveKinematics.toSwerveModuleStates(speeds);
                     DRIVETRAIN.setModuleSpeeds(states);
                  }, this)
                        // 3. Stop when X position > target
                        .until(() -> getPose().getX() >= targetDistanceMeters))
            .finallyDo(() -> {
               // 4. Stop the robot when done
               drive(0, 0, 0, false); // You might need to create a simple helper for stopping or send 0 speeds
            });
   }

   public Command spinSteerMotors(double spins) {
      return Commands.runOnce(() -> {
         // 1080 degrees = 3.0 Rotations
         DRIVETRAIN.spinAllSteerMotors(spins);
      }, this);
   }
}