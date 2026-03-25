package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import frc.robot.Util.Dashboard;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Limelight;

public class RobotContainer {

  public Shooter SHOOTER;
  public Indexer INDEXER;
  public Dashboard DASHBOARD;
  public Intake INTAKE;
  public SwerveDrive SWERVE;
  public DriveTrain DRIVE_TRAIN;
  public Limelight LIMELIGHT;
  public Climber CLIMBER;

  public Trigger shooterInRange;

  // 2. Initialize the Controller
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_secondController = 
    new CommandXboxController(OperatorConstants.kSecondControllerPort);


  private final SendableChooser<Command> autoChooser;
  //private static SendableChooser<Command> autoBranchChooser;
  RobotConfig config;


  public RobotContainer() {
    SWERVE = SwerveDrive.getInstance();
    SHOOTER = Shooter.getInstance();
    INDEXER = Indexer.getInstance();
    DASHBOARD = new Dashboard();
    INTAKE = Intake.getInstance();
    DRIVE_TRAIN = DriveTrain.getInstance();
    LIMELIGHT = Limelight.getInstance("limelight-robot");
    CLIMBER = Climber.getInstance();

    //semi-auto triggers
    shooterInRange = new Trigger(SHOOTER.inRange());

    // register named commands
    NamedCommands.registerCommand("aimOn", SHOOTER.aimOn());
    NamedCommands.registerCommand("aimOff", SHOOTER.aimOff());
    NamedCommands.registerCommand("intakeOn", INTAKE.intakeOn());
    NamedCommands.registerCommand("intakeOff", INTAKE.intakeOff());
    NamedCommands.registerCommand("shooterTest", SHOOTER.testShooter(0.25));
    NamedCommands.registerCommand("turretTest", SHOOTER.testTurretAngle(-180));
    NamedCommands.registerCommand("shooterOff", SHOOTER.shooterOff());
    NamedCommands.registerCommand("feederOn", SHOOTER.feederOn());
    NamedCommands.registerCommand("feederOff", SHOOTER.feederOff());
    NamedCommands.registerCommand("indexerOn", INDEXER.indexerOn());
    NamedCommands.registerCommand("indexerOff", INDEXER.indexerOff());
    // 3. Set Default Command for Driving
    // We pass the joystick inputs to the subsystem's drive method.
    // Note: Xbox Left Y is usually Forward (X), Left X is Strafe (Y), Right X is Rotation.
    SWERVE.setDefaultCommand(
      SWERVE.joystickDrive(
          () -> -m_driverController.getLeftY(), // Map Y (forward) to xSpeed
          () -> -m_driverController.getLeftX(), // Map X (strafe) to ySpeed
          () -> -m_driverController.getRightX()
      )
  );

    configureBindings();


    loadPaths();
    autoChooser = AutoBuilder.buildAutoChooser();
    //autoBranchChooser = AutoBuilder.buildAutoChooser();b

    SmartDashboard.putData("Autonomous", autoChooser);
  }


  private void configureBindings() {
    // 4. Reset Gyro Binding (Start Button)
      m_driverController.start().onTrue(SWERVE.resetGyro());

    // 5. Slow Mode Binding (Left Bumper)
    // While held, slow mode is active; when released, it returns to normal.
      m_driverController.leftBumper()
          .onTrue(Commands.runOnce(() -> SWERVE.setSlowmode(true)))
          .onFalse(Commands.runOnce(() -> SWERVE.setSlowmode(false)));
    
    //m_driverController.rightTrigger().onTrue(SHOOTER.shooterOn());
    //m_driverController.rightTrigger().onFalse(SHOOTER.shooterOff());

    m_driverController.leftTrigger().onTrue(SHOOTER.shooterOn());
    m_driverController.rightTrigger().onTrue(SHOOTER.feederOn());
    m_driverController.rightTrigger().onTrue(INDEXER.indexerOn());
    m_driverController.leftTrigger().onFalse(SHOOTER.shooterOff());
    m_driverController.leftTrigger().onFalse(INDEXER.indexerOff());
    m_driverController.leftTrigger().onFalse(SHOOTER.feederOff());

    //m_driverController.leftTrigger().onTrue(new ParallelCommandGroup(SHOOTER.feederOn(), INDEXER.indexerOn()));
    //m_driverController.leftTrigger().onFalse(new ParallelCommandGroup(SHOOTER.feederOff(), INDEXER.indexerOff()));

    //m_driverController.pov(90).onTrue(SHOOTER.rotateRight());
    //m_driverController.pov(90).onFalse(SHOOTER.rotateStopCommand());
    //m_driverController.pov(270).onTrue(SHOOTER.rotateLeft());
    //m_driverController.pov(270).onFalse(SHOOTER.rotateStopCommand());

    //m_secondController.x().onTrue(INTAKE.intakeFlipIn());
    m_secondController.b().onTrue(INTAKE.intakeFlipOut());

    m_driverController.x().onTrue(INTAKE.intakeOn());
    m_driverController.y().onTrue(INTAKE.intakeOff());
    m_secondController.a().toggleOnTrue(LIMELIGHT.updateRobotPosition());

    //m_driverController.a().onTrue(CLIMBER.climb());
    //m_driverController.b().onTrue(CLIMBER.unclimb());

    //m_driverController.a().onTrue(SHOOTER.aimOn());
    //m_driverController.b().onTrue(SHOOTER.aimOff());

    //m_driverController.rightBumper().onTrue(DRIVE_TRAIN.resetSteer());

    // m_driverController.povUp().onTrue(INTAKE.nudgeUp());
    // m_driverController.povDown().onTrue(INTAKE.nudgeDown());

    //m_driverController.povDownLeft().onTrue(SWERVE.driveWheelSpins(3));


    //second controller commands
    
    //m_secondController.y().toggleOnTrue(SHOOTER.setTarget("HUB"));
    //m_secondController.a().toggleOnTrue(SHOOTER.setTarget("DEPOT"));
    //m_secondController.b().toggleOnTrue(SHOOTER.setTarget("OUTPOST"));
    m_secondController.leftBumper().toggleOnTrue(SHOOTER.aimOn());
    m_secondController.rightBumper().toggleOnTrue(SHOOTER.aimOff());

    /*
    m_secondController.povDown().onTrue(CLIMBER.setState("IDLE"));
    m_secondController.povLeft().onTrue(CLIMBER.setState("L1"));
    m_secondController.povRight().onTrue(CLIMBER.setState("L2"));
    m_secondController.povUp().onTrue(CLIMBER.setState("L3"));
    */

    
    m_secondController.leftTrigger().toggleOnTrue(CLIMBER.testClimb());
    m_secondController.rightTrigger().toggleOnTrue(CLIMBER.testUnClimb());
    m_secondController.leftTrigger().toggleOnFalse(CLIMBER.stopClimb());
    m_secondController.rightTrigger().toggleOnFalse(CLIMBER.stopClimb());


    m_secondController.povUp().toggleOnTrue(SHOOTER.adjustShooterSpeed(0.02));
    m_secondController.povDown().toggleOnTrue(SHOOTER.adjustShooterSpeed(-0.02));
    /* 
    m_secondController.povLeft().toggleOnTrue(SHOOTER.adjustShooterRotation(Math.toRadians(2)));
    m_secondController.povRight().toggleOnTrue(SHOOTER.adjustShooterRotation(Math.toRadians(-2)));

    m_secondController.x().toggleOnTrue(SHOOTER.toggleAutoShooting());
*/
    //turret flywheel tests for feedforward tuning
    //m_secondController.leftTrigger().onTrue(SHOOTER.shooterOn());
    //m_secondController.leftTrigger().onFalse(SHOOTER.shooterOff());
    //m_secondController.y().toggleOnTrue(SHOOTER.testNewPIDValues());
    //m_secondController.a().toggleOnTrue(LIMELIGHT.updateRobotPosition());

    //turret turning tests for feedforward tuning
    //m_secondController.povLeft().toggleOnTrue(SHOOTER.testTurretAngle(90));
    //m_secondController.povRight().toggleOnTrue(SHOOTER.testTurretAngle(-90));

    //semi-autonomous shooting
    /*
    shooterInRange.onTrue(SHOOTER.shootAndFeed());
    shooterInRange.onFalse(new ParallelCommandGroup(SHOOTER.shooterOff(), SHOOTER.feederOff()));
    */
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
            if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
              return true;
            } else {
              return false;
            }
        //return false;
      },  
      SWERVE 
    );

  }
  
  public void report() {

    /*m_driverController.leftBumper().onTrue(SHOOTER.rotateTurret(90));
    m_driverController.rightBumper().onTrue(SHOOTER.rotateTurret(-90));*/
  }

  public void periodic() {
    SHOOTER.periodic();
    SHOOTER.report();
    DASHBOARD.periodic();
    INTAKE.report();
    LIMELIGHT.report();
  }
  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
