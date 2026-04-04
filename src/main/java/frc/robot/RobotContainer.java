package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

import frc.robot.subsystems.*;
import frc.robot.Util.Dashboard;

public class RobotContainer {

  public Shooter SHOOTER;
  public Indexer INDEXER;
  public Dashboard DASHBOARD;
  public Intake INTAKE;
  public SwerveDrive SWERVE;
  public DriveTrain DRIVE_TRAIN;
  public Limelight LIMELIGHT;
  //public Climber CLIMBER;

  public Trigger shooterInRange;

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_secondController = new CommandXboxController(OperatorConstants.kSecondControllerPort);
  //private final CommandXboxController m_testController = new CommandXboxController(OperatorConstants.kTestControllerPort);

  private final SendableChooser<Command> autoChooser;
  RobotConfig config;


  public RobotContainer() {
    SWERVE = SwerveDrive.getInstance();
    SHOOTER = Shooter.getInstance();
    INDEXER = Indexer.getInstance();
    DASHBOARD = new Dashboard();
    INTAKE = Intake.getInstance();
    DRIVE_TRAIN = DriveTrain.getInstance();
    LIMELIGHT = Limelight.getInstance("limelight-robot");
    //CLIMBER = Climber.getInstance();

    //semi-auto triggersi
    shooterInRange = new Trigger(SHOOTER.inRange());

    NamedCommands.registerCommand("intakeOn", INTAKE.intakeOn());
    NamedCommands.registerCommand("intakeOff", INTAKE.intakeOff());
    NamedCommands.registerCommand("shooterTest", SHOOTER.testShooter(0.25));
    NamedCommands.registerCommand("turretTest", SHOOTER.testTurretAngle(-180));
    NamedCommands.registerCommand("shooterOff", SHOOTER.shooterOff());
    NamedCommands.registerCommand("feederOn", SHOOTER.feederOn());
    NamedCommands.registerCommand("feederOff", SHOOTER.feederOff());
    NamedCommands.registerCommand("indexerOn", INDEXER.indexerOn());
    NamedCommands.registerCommand("indexerOff", INDEXER.indexerOff());
    NamedCommands.registerCommand("shooterOn", SHOOTER.shooterOn());
    NamedCommands.registerCommand("aimOn", SHOOTER.aimOn());
    NamedCommands.registerCommand("targetHub", SHOOTER.setTarget("HUB"));
    NamedCommands.registerCommand("intakeOut", INTAKE.intakeFlipOut());
    //NamedCommands.registerCommand("climberOut", CLIMBER.unclimb());
    //NamedCommands.registerCommand("climberIn", CLIMBER.climb());
    NamedCommands.registerCommand("autoShoot", SHOOTER.shooterOnDefault());


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

    SmartDashboard.putData("Autonomous", autoChooser);
  }


  private void configureBindings() {
      
    // --- DRIVER CONTROLLER ---
    m_driverController.start().onTrue(SWERVE.resetGyro());

    m_driverController.x().onTrue(INTAKE.intakeOn());
    m_driverController.y().onTrue(INTAKE.intakeOff());

    m_driverController.a().toggleOnTrue(new SequentialCommandGroup(
                                  SHOOTER.shooterOn(), 
                                  new WaitCommand(0.4), 
                                  new ParallelCommandGroup(SHOOTER.feederOn(), INDEXER.indexerOn(), INTAKE.intakeOn())
                                  ));

    m_driverController.b().toggleOnTrue(new ParallelCommandGroup(
      SHOOTER.shooterOff(),
      SHOOTER.feederOff(),
      INDEXER.indexerOff(),
      INTAKE.intakeOff()
    ));  

    m_driverController.leftBumper().toggleOnTrue(SWERVE.toggleSlowMode());
    m_driverController.povLeft().toggleOnTrue(SWERVE.resetLeftTrench());
    m_driverController.povRight().toggleOnTrue(SWERVE.resetRightTrench());
    
    // --- SECOND CONTROLLER ---

    //m_secondController.rightTrigger().toggleOnTrue(SHOOTER.shooterOn());
    //m_secondController.rightTrigger().toggleOnTrue(SHOOTER.feederOn());
    //m_secondController.rightTrigger().toggleOnTrue(INDEXER.indexerOn());

    //m_secondController.rightTrigger().toggleOnFalse(SHOOTER.shooterOff());
    //m_secondController.rightTrigger().toggleOnFalse(SHOOTER.feederOff());
    //m_secondController.rightTrigger().toggleOnFalse(INDEXER.indexerOff());

    //m_secondController.y().toggleOnTrue(CLIMBER.climb());
    //m_secondController.x().toggleOnTrue(CLIMBER.unclimb());

    m_secondController.a().toggleOnTrue(SHOOTER.aimOn());
    m_secondController.b().toggleOnTrue(SHOOTER.adjustShooterSpeed(0.01));
    m_secondController.b().toggleOnTrue(SHOOTER.adjustShooterSpeed(-0.01));
    //m_secondController.b().toggleOnTrue(SWERVE.toggleVision());
    m_secondController.x().toggleOnTrue(INTAKE.intakeFlipOut());

    //m_secondController.povUp().toggleOnTrue(SHOOTER.adjustShooterSpeed(0.02));
    //m_secondController.povDown().toggleOnTrue(SHOOTER.adjustShooterSpeed(-0.02));

    m_secondController.povUp().toggleOnTrue(SHOOTER.setTarget("HUB"));
    m_secondController.povLeft().toggleOnTrue(SHOOTER.setTarget("DEPOT"));
    m_secondController.povRight().toggleOnTrue(SHOOTER.setTarget("OUTPOST"));

    //m_secondController.povLeft().toggleOnTrue(INTAKE.intakeFlipOut());
    //m_secondController.b().toggleOnTrue(LIMELIGHT.updateRobotPosition());

    // m_secondController.leftBumper().toggleOnTrue(CLIMBER.testClimb());
    // m_secondController.leftBumper().toggleOnFalse(CLIMBER.stopClimb());
    // m_secondController.rightBumper().toggleOnTrue(CLIMBER.testUnClimb());
    // m_secondController.rightBumper().toggleOnFalse(CLIMBER.stopClimb());R.climb());
    //m_secondController.leftBumper().toggleOnTrue(CLIMBER.testClimb());
    //m_secondController.leftBumper().toggleOnFalse(CLIMBER.stopClimb());
    //m_secondController.rightBumper().toggleOnTrue(CLIMBER.testUnClimb());
   // m_secondController.rightBumper().toggleOnFalse(CLIMBER.stopClimb());


    m_secondController.leftTrigger().toggleOnTrue(SHOOTER.adjustShooterRotation(1));
    m_secondController.rightTrigger().toggleOnTrue(SHOOTER.adjustShooterRotation(-1));

    // --- TEST CONTROLLER ---

    //m_testController.leftBumper().toggleOnTrue(CLIMBER.testClimb());
    //m_testController.leftBumper().toggleOnFalse(CLIMBER.stopClimb());
    //m_testController.rightBumper().toggleOnTrue(CLIMBER.testUnClimb());
    //m_testController.rightBumper().toggleOnFalse(CLIMBER.stopClimb());
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
      },  
      SWERVE 
    );
  }
  
  public void periodic() {
    SHOOTER.periodic();
    SHOOTER.report();
    DASHBOARD.periodic();
    INTAKE.report();
    LIMELIGHT.report();
    //CLIMBER.report();
  }
  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}