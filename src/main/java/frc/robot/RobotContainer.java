package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveDrive;

public class RobotContainer {
  // 1. Initialize the Swerve Subsystem
  private final SwerveDrive m_swerveDrive = SwerveDrive.getInstance();

  // 2. Initialize the Controller
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
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
  }

  private void configureBindings() {
    // 4. Reset Gyro Binding (Start Button)
    m_driverController.start().onTrue(m_swerveDrive.resetGyro());

    // 5. Slow Mode Binding (Left Bumper)
    // While held, slow mode is active; when released, it returns to normal.
    m_driverController.leftBumper()
        .onTrue(Commands.runOnce(() -> m_swerveDrive.setSlowmode(true)))
        .onFalse(Commands.runOnce(() -> m_swerveDrive.setSlowmode(false)));
  }

  public Command getAutonomousCommand() {
    // For now, return a command that does nothing (or your auto routine)
    return Commands.print("No autonomous command configured");
  }
}