package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.Constants.SwerveConstants;

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
            new Rotation2d(Math.toRadians(gyro.getAngle())), // 2. Gyro Angle
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

      chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 0, new Rotation2d(Math.toRadians(gyro.getAngle())));
   }

   @Override
   public void periodic() {
      // Update odometry with current gyro heading and module positions (from Phoenix
      // 6)
      m_odometry.update(new Rotation2d(Math.toRadians(gyro.getAngle())), DRIVETRAIN.getModulePositions());

      SmartDashboard.putNumber("Robot X", getPose().getX());
      SmartDashboard.putNumber("Robot Y", getPose().getY());
      SmartDashboard.putNumber("Robot Gyro Angle", gyro.getAngle());
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

      resetOdometry(new Pose2d(getPose().getTranslation(), matchStartRotation));
   }

   public Command resetGyro() {
      return Commands.runOnce(() -> {
         Rotation2d matchStartRotation = isRedAlliance() ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);

         gyro.reset();
         gyro.setAngleAdjustment(matchStartRotation.getDegrees());

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
}