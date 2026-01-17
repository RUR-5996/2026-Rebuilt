package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;


public class RobotContainer {

  public LEDs LEDS;
  public SwerveDrive SWERVE;

  // 1. Initialize the Swerve Subsystem
  private final SwerveDrive m_swerveDrive = SwerveDrive.getInstance();

  // 2. Initialize the Controller
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);


  private final SendableChooser<Command> autoChooser;
  private static SendableChooser<Command> autoBranchChooser;
  RobotConfig config;


  public RobotContainer() {

    LEDS = LEDs.getInstance();
    SWERVE = SwerveDrive.getInstance();

    // 3. Set Default Command for Driving
    // We pass the joystick inputs to the subsystem's drive method.
    // Note: Xbox Left Y is usually Forward (X), Left X is Strafe (Y), Right X is Rotation.
    m_swerveDrive.setDefaultCommand(
      m_swerveDrive.joystickDrive(
          () -> -m_driverController.getLeftY(), // Map Y (forward) to xSpeed
          () -> -m_driverController.getLeftX(), // Map X (strafe) to ySpeed
          () -> -m_driverController.getRightX()
      )
  );

    configureBindings();


    loadPaths();
    autoChooser = AutoBuilder.buildAutoChooser();
    autoBranchChooser = AutoBuilder.buildAutoChooser();
  }


    private void configureBindings() {
    // 4. Reset Gyro Binding (Start Button)
      m_driverController.start().onTrue(m_swerveDrive.resetGyro());

    // 5. Slow Mode Binding (Left Bumper)
    // While held, slow mode is active; when released, it returns to normal.
      m_driverController.leftBumper()
          .onTrue(Commands.runOnce(() -> m_swerveDrive.setSlowmode(true)))
          .onFalse(Commands.runOnce(() -> m_swerveDrive.setSlowmode(false)));

      m_driverController.a().onTrue(LEDS.changeColor());
    }


    private void loadPaths() {
      try {
        config = RobotConfig.fromGUISettings();
      } catch (Exception e) {
          // Handle exception as needed
          e.printStackTrace();
      }

    AutoBuilder.configure(
      SWERVE::getOdometryPose,
      SWERVE::resetOdometry,
      SWERVE::getActualSpeeds,
      (speeds, feedforwards) -> SWERVE.setAutoChassisSpeeds(speeds),
      SwerveConstants.autoConfig,
      config,
      () -> {
            /*if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
              return true;
            } else {
              return false;
            }*/
        return false;
      },  
      SWERVE 
    );
  }

  public Command getAutonomousCommand() {
    // For now, return a command that does nothing (or your auto routine)
    return autoChooser.getSelected();
  }
}
