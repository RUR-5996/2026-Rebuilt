package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveDrive extends SubsystemBase {

   public final DriveTrain DRIVETRAIN;
   private static SwerveDrive SWERVE;

   private boolean slowmode = false;
   private boolean holdAngleEnabled = false;
   private double holdAngle = 0;

   private final SwerveDrivePoseEstimator m_odometry;
   private final AHRS gyro;

   private final PIDController rotationController;

   public SwerveDrive() {
      DRIVETRAIN = DriveTrain.getInstance();

      gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
      gyro.reset();

      // The 2025/2026 Pose Estimator constructor
      m_odometry = new SwerveDrivePoseEstimator(
         DRIVETRAIN.swerveKinematics,   // 1. Kinematics
         getHeading(),           // 2. Gyro Angle
         DRIVETRAIN.getModulePositions(), // 3. Module Positions
         new Pose2d()           // 4. Initial Pose
      );

      rotationController = new PIDController(
         SwerveConstants.steerKP, // Use your constants here
         SwerveConstants.steerKI, 
         SwerveConstants.steerKD
      );
      rotationController.enableContinuousInput(-180, 180);
      rotationController.setTolerance(2);
   }

   @Override
   public void periodic() {
      // Update odometry with current gyro heading and module positions (from Phoenix 6)
      m_odometry.update(getHeading(), DRIVETRAIN.getModulePositions());

      SmartDashboard.putNumber("Robot X", getPose().getX());
      SmartDashboard.putNumber("Robot Y", getPose().getY());
      SmartDashboard.putNumber("Robot Gyro Angle", getHeading().getDegrees());
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
         // Apply deadband and scaling
         double xSpeed = MathUtil.applyDeadband(lx.getAsDouble(), 0.1) * SwerveConstants.MAX_SPEED_METERS_PER_SECOND;
         double ySpeed = MathUtil.applyDeadband(ly.getAsDouble(), 0.1) * SwerveConstants.MAX_SPEED_METERS_PER_SECOND;
         double rot;

         if (holdAngleEnabled) {
            rot = rotationController.calculate(getHeading().getDegrees(), holdAngle);
         } else {
            rot = MathUtil.applyDeadband(rx.getAsDouble(), 0.1) * SwerveConstants.MAX_ANGULAR_SPEED;
         }

         if (slowmode) {
            xSpeed *= DriverConstants.PRECISION_RATIO;
            ySpeed *= DriverConstants.PRECISION_RATIO;
            rot *= DriverConstants.PRECISION_RATIO;
         }

         // Create ChassisSpeeds (Field Relative)
         // Note: ly is often forwarded/backward (X in robot frame), lx is strafe (Y in robot frame)
         ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rot, getHeading()
         );

         // Convert to module states and desaturate
         SwerveModuleState[] states = DRIVETRAIN.swerveKinematics.toSwerveModuleStates(speeds);
         SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_SPEED_METERS_PER_SECOND);

         DRIVETRAIN.setModuleSpeeds(states);
      }, this);
   }

   // --- Helpers ---

   public Rotation2d getHeading() {
      return gyro.getRotation2d();
   }

   public Pose2d getPose() {
      return m_odometry.getEstimatedPosition();
   }

   public void resetOdometry(Pose2d pose) {
      m_odometry.resetPosition(getHeading(), DRIVETRAIN.getModulePositions(), pose);
   }

   public Command resetGyro() {
      return Commands.runOnce(() -> {
         gyro.reset();
         // Resetting gyro usually requires resetting odometry to keep them synced
         resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d()));
      });
   }

   public void setSlowmode(boolean enabled) {
      this.slowmode = enabled;
   }

   public boolean isRedAlliance() {
      var alliance = DriverStation.getAlliance();
      return alliance.isPresent() && alliance.get() == Alliance.Red;
   }
}