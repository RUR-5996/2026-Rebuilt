// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kSecondControllerPort = 1;
  }

  public static class SwerveConstants {
    public static final double driveKP = 1;
    public static final double driveKI = 0;
    public static final double driveKD = 0;

    public static final double CURRENT_LIMIT = 40;

    public static final double steerKP = 60;
    public static final double steerKI = 0;
    public static final double steerKD = 0;

    public static final double DRIVE_MOTOR_GEARING = 5.355;
    public static final double STEER_MOTOR_GEARING = 21.43;
    public static final double WHEEL_RADIUS_METERS = 0.10/2;
    public static final double FALCON_RPM = 6379.0;
    public static final double DRIVE_FACTOR = FALCON_RPM / (60.0 * DRIVE_MOTOR_GEARING) * 2 * Math.PI * WHEEL_RADIUS_METERS; //mps

    public static final double WHEEL_BASE_WIDTH = 0.517;
    public static final double TRACK_WIDTH = 0.516;

    public static final double METERS_PER_ROBOT_REVOLUTION = 2 * Math.PI * Math.hypot(TRACK_WIDTH, WHEEL_BASE_WIDTH);
    // public static final double MAX_SPEED_METERS_PER_SECOND = DRIVE_FACTOR;        
    // public static final double MAX_ANGULAR_SPEED = (MAX_SPEED_METERS_PER_SECOND / METERS_PER_ROBOT_REVOLUTION) * (2 * Math.PI);

    public static final boolean FL_STEER_INVERT = true;
    public static final boolean FR_STEER_INVERT = true;
    public static final boolean BL_STEER_INVERT = true;
    public static final boolean BR_STEER_INVERT = true;

    public static final InvertedValue FL_DRIVE_INVERT_TYPE = InvertedValue.Clockwise_Positive;
    public static final InvertedValue FR_DRIVE_INVERT_TYPE = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue RL_DRIVE_INVERT_TYPE = InvertedValue.Clockwise_Positive;
    public static final InvertedValue RR_DRIVE_INVERT_TYPE = InvertedValue.CounterClockwise_Positive;

    public static final boolean FL_STEER_INVERT_TYPE = true;
    public static final boolean FR_STEER_INVERT_TYPE = true;
    public static final boolean RL_STEER_INVERT_TYPE = true;
    public static final boolean RR_STEER_INVERT_TYPE = true;

    public static final double FL_STEER_OFFSET = 0;
    public static final double FR_STEER_OFFSET = 0;
    public static final double RL_STEER_OFFSET = 0;
    public static final double RR_STEER_OFFSET = 0;

    // --- Speed Constants ---
    public static final double MAX_SPEED_METERS_PER_SECOND = 3; // 4.5
    public static final double MAX_ANGULAR_SPEED = (450.0 /360.0) * (2 * Math.PI);


    public static final PPHolonomicDriveController autoConfig = new PPHolonomicDriveController(
      new PIDConstants(7.5, 0, 0), //drive
      new PIDConstants(5, 0, 0)); //steer //TODO change constants so the robot moves accurately
  }

  public final class ShooterConstants {

    public static final double DEFAULT_SHOOTER_SPEED = 0.65;
    public static final double FEEDER_VELOCITY = 50;

    public static final int INDEXER_MOTOR_ID = 12;
    public static final int FEEDER_MOTOR_ID = 13;
    public static final int POWER_MOTOR_1_ID = 14;
    public static final int POWER_MOTOR_2_ID = 15;

    public static final double POWER_MOTOR_P = 0.0;
    public static final double POWER_MOTOR_I = 0.0;
    public static final double POWER_MOTOR_D = 0.0;
    public static final double POWER_MOTOR_V = 0.00175;
    public static final double POWER_MOTOR_GEAR_RATIO = 37.0/27.0;
    public static final double NEO_MAX_RPM = 5600.0;

    public static final double FEEDER_MOTOR_P = 1.0;
    public static final double FEEDER_MOTOR_I = 0.0;
    public static final double FEEDER_MOTOR_D = 0.0;
    public static final double FEEDER_MOTOR_V = 0.12;

    public static final double INDEXER_MOTOR_P = 1.0;
    public static final double INDEXER_MOTOR_I = 0.0;
    public static final double INDEXER_MOTOR_D = 0.0;
    public static final double INDEXER_MOTOR_V = 0.12;  

    public static final int TURRET_CANCODER_ID = 11; 
    public static final int TURRET_MOTOR_ID = 20; 
    
    public static final double TURRET_MOTOR_P = 0.01;
    public static final double TURRET_MOTOR_I = 0.00000005;
    public static final double TURRET_MOTOR_D = 0.01;
    public static final double TURRET_MOTOR_V = 0.1;

    public static final double MOTOR_TO_TURRET_RATIO = (14.0/45.0) * (23.0/123.0)*360.0; //(1/20)*(23/123)
    public static final double CANCODER_TO_TURRET_RATIO = (30.0/45.0) * (23.0/123.0); //(1/20)*(45/30)
    public static final double MAX_TURRET_ANGLE = 180.0; //feederV2 +360 TODO replace with actual value
    public static final double MIN_TURRET_ANGLE = -180.0; //feederV2 -270 TODO replace with actual value

    public static final double TURRET_X = -0.1373;
    public static final double TURRET_Y = 0.1438;
    public static final double VEC_TURRET_LEN = Math.sqrt(Math.pow(TURRET_X, 2) + Math.pow(TURRET_Y, 2));
    public static final double VEC_TURRET_PHI = Math.acos(TURRET_X / TURRET_Y);

    public static final double MINIMUM_SHOOTING_DISTANCE = 3.0; //TODO replace with actual value

    public static final double TIME_TO_SHOOT = 1.0; //TODO replace with actual value
  }

  public final class DriverConstants {
    // --- Slow Mode / Precision Mode ---
    // When slow mode is toggled, speeds are multiplied by this ratio.
    public static final double PRECISION_RATIO = 0.35;

  }

  public static class IntakeConstants{
    public static final int flipOutMotorLId = 41;
    public static final int flipOutMotorRId = 42;
    public static final int powerMotorId = 43;

    public static final double POS_IN = 0;

    public static final double SPEED = 0.5;
    public static final double POS_OUT = 90;
    public static final double FLIPOUT_COEFFICIENT = 1/15.0*18.0/42.0*1/360.0;
  }

  public final class LimelightConstants {
    public static final String ROBOT_LIMELIGHT_NAME = "robot";
    public static final Transform3d ROBOT_LIMELIGHT_OFFSET = new Transform3d(
      new Translation3d( // in metres
          0.2728,   // X (+ is forward)
          0.2489,   // Y (+ is left)
          0.2189   // Z (+ is up)
      ),
      new Rotation3d( //in radians
        0.0, // roll
        Math.toRadians(60.0), // pitch (+ is up)
        0.0 // yaw
      ));


    public static final String TURRET_LIMELIGHT_NAME = "turret";

    public static final double MAX_VISION_SPIN = 720; // degrees per second TODO test if good
  }

  public static class ColorConstants {
    public static final double RAINBOW = -.99;
    public static final double VIOLET = .91;
    public static final double PINK = .57;
    public static final double FLASHBANG = .93;
    public static final double BLUEGREEN = .79;
    public static final double LIGHTBLUE = .83;
    public static final double DARKBLUE = .85;
    public static final double BLUEPINK = .89;
    public static final double STROBERED = -.11;
    public static final double STROBEBLUE = -.09;
    public static final double STROBEGOLD = -.07;
    public static final double STROBEWHITE = -.05;
  }
}
