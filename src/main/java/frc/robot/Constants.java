// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

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
  }
  public static class SwerveConstants {
    public static final double driveKP = 0.5;
    public static final double driveKI = 0;
    public static final double driveKD = 0;
    public static final double deriveKIzone = 300;

    public static final double LOOP_RAMP_TIME = 0.3;
    public static final double CURRENT_LIMIT = 30;
    public static final double ROTOR_SENSOR_RAT = 1;

    public static final double steerKP = 50;
    public static final double steerKI = 0;
    public static final double steerKD = 0;

    public static final double DRIVE_MOTOR_GEARING = 5.355;
    public static final double STEER_MOTOR_GEARING = 21.43;
    public static final double STEER_MOTOR_COEFFICIENT = 1.0 / STEER_MOTOR_GEARING * 360.0;
    public static final double WHEEL_RADIUS_METERS = 0.10/2;
    public static final double FALCON_RPM = 6379.0;
    public static final double DRIVE_FACTOR = FALCON_RPM / (60.0 * DRIVE_MOTOR_GEARING) * 2 * Math.PI * WHEEL_RADIUS_METERS; //mps

    public static final double WHEEL_BASE_WIDTH = 0.517;
    public static final double TRACK_WIDTH = 0.516;

    public static final double SECONDSper100MS = .1;
    public static final double TICKSperTALONFX_Rotation = 2048;
    public static final double DRIVE_MOTOR_TICKSperREVOLUTION = DRIVE_MOTOR_GEARING * TICKSperTALONFX_Rotation;
    public static final double METERSperWHEEL_REVOLUTION = 2 * Math.PI * WHEEL_RADIUS_METERS;
    public static final double METERSperROBOT_REVOLUTION = 2 * Math.PI
            * Math.hypot(TRACK_WIDTH, WHEEL_BASE_WIDTH);
    public static final double MAX_SPEED_METERSperSECOND = DRIVE_FACTOR;        
    public static final double MAX_SPEED_RADIANSperSECOND = MAX_SPEED_METERSperSECOND / METERSperROBOT_REVOLUTION
    * (2 * Math.PI);
    public static final double P_ROTATION_CONTROLLER = 0.055;
    public static final double I_ROTATION_CONTROLLER = 0.0;
    public static final double D_ROTATION_CONTROLLER = 0.0;

    // public static final Translation2d FL_LOC = new Translation2d(SwerveConstants.WHEEL_BASE_WIDTH / 2, SwerveConstants.TRACK_WIDTH / 2);
    // public static final Translation2d FR_LOC = new Translation2d(SwerveConstants.WHEEL_BASE_WIDTH / 2, -SwerveConstants.TRACK_WIDTH / 2);
    // public static final Translation2d RL_LOC = new Translation2d(-SwerveConstants.WHEEL_BASE_WIDTH / 2, SwerveConstants.TRACK_WIDTH / 2);
    // public static final Translation2d RR_LOC = new Translation2d(-SwerveConstants.WHEEL_BASE_WIDTH / 2, -SwerveConstants.TRACK_WIDTH / 2);

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
    public static final double MAX_SPEED_METERS_PER_SECOND = 4; // 4.5
    public static final double MAX_ANGULAR_SPEED = 2 * Math.PI;
  }

  public final class DriverConstants {

    // --- Controller Ports ---
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // --- Deadbands ---
    public static final double CONTROLLER_DEADBAND = 0.1;

    // --- Drive Scaling (Governors) ---
    // 1.0 = 100% speed, 0.5 = 50% speed. 
    // Use these to keep the robot manageable during testing.
    public static final double DRIVE_GOVERNOR = 0.85; 
    public static final double TURN_GOVERNOR = 0.70;

    // --- Slow Mode / Precision Mode ---
    // When slow mode is toggled, speeds are multiplied by this ratio.
    public static final double PRECISION_RATIO = 0.35;

    // --- Slew Rate Limiters ---
    // Higher values = more "snappy", lower values = smoother acceleration.
    // Units are units-per-second (e.g., 3.0 means it takes 0.33s to reach full speed).
    public static final double DRIVE_SLEW_RATE = 3.0;
    public static final double TURN_SLEW_RATE = 4.0;
  }

  public static class IntakeConstants{ //TODO fill in
    public static final int flipOutMotor1Id = 0;
    public static final int flipOutMotor2Id = 0;
    public static final int powerMotorId = 0;
  }
}
