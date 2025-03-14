package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SuperstructureCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.rollers.RollerSystemIOMinion;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.beak.Beak;
import frc.robot.subsystems.superstructure.beak.BeakIOMinion;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.superstructure.extender.Extender;
import frc.robot.subsystems.superstructure.extender.ExtenderIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Superstructure superstructure;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.kWarning);
  // private final LoggedNetworkNumber endgameAlert1 =
  //     new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #1", 30.0);
  // private final LoggedNetworkNumber endgameAlert2 =
  //     new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #2", 15.0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    Elevator elevator = null;
    Extender extender = null;
    Beak beak = null;

    switch (Constants.getMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        elevator = new Elevator(new ElevatorIOTalonFX());
        extender =
            new Extender(
                new ExtenderIOTalonFX(),
                new RollerSystemIOMinion(16, "rio", 40, false, true, (5.0 * 5.0)));
        beak =
            new Beak(new BeakIOMinion(), new RollerSystemIOMinion(15, "rio", 40, false, true, 1));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }
    if (elevator == null) {
      elevator = new Elevator(new ElevatorIO() {});
    }

    superstructure = new Superstructure(elevator, extender, beak);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    NamedCommands.registerCommand("HomeE", elevator.homingSequence());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    // Lock to 0° when A button is held
    driver
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    driver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    superstructure.setDefaultCommand(
        SuperstructureCommands.joystickMoveAll(
            superstructure,
            () -> -operator.getRightY(),
            () -> -operator.getLeftY(),
            () -> -operator.getLeftX()));

    // Operator controls

    // Game piece manipulation
    // operator
    //     .rightBumper()
    //     .whileTrue(Commands.run(superstructure::startManipulatingGamePieces, superstructure))
    //     .onFalse(Commands.run(superstructure::stopManipulatingGamePieces, superstructure));

    // operator.leftBumper().onTrue(Commands.run(superstructure::ejectCoral, superstructure));
    // operator.leftTrigger().onTrue(Commands.run(superstructure::ejectAlgae, superstructure));

    // // Elevator
    operator.a().onTrue(Commands.runOnce(superstructure::sendElevatorHome, superstructure));
    operator.y().onTrue(Commands.runOnce(superstructure::sendElevatorToPercent, superstructure));
    operator.x().onTrue(superstructure.homeElevator());
    // operator.b().onTrue(superstructure.homeExtender());

    // Extender
    operator
        .leftBumper()
        .onTrue(Commands.runOnce(superstructure::sendExtenderHome, superstructure));
    operator
        .rightBumper()
        .onTrue(Commands.runOnce(superstructure::sendExtenderToHor, superstructure));

    // Beak
    // operator.y().onTrue(Commands.runOnce(superstructure::sendBeakHome, superstructure));
    // operator.b().onTrue(Commands.runOnce(superstructure::sendBeakToMax, superstructure));
  }

  // Creates controller rumble command
  private Command controllerRumbleCommand() {
    return Commands.startEnd(
        () -> {
          driver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
          operator.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        },
        () -> {
          driver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
          operator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        });
  }

  // Update dashboard data
  public void updateDashboardOutputs() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  public void updateAlerts() {
    // Controller disconnected alerts
    driverDisconnected.set(!DriverStation.isJoystickConnected(driver.getHID().getPort()));
    operatorDisconnected.set(!DriverStation.isJoystickConnected(operator.getHID().getPort()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return AutoBuilder.followPath(PathPlannerPath.fromPathFile("Just Park"));
    return autoChooser.get();
  }
}
