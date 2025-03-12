package frc.robot.subsystems.superstructure.extender;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.Robot;
import frc.robot.subsystems.rollers.RollerSystemIO;
import frc.robot.subsystems.rollers.RollerSystemIOInputsAutoLogged;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Extender {
  public static final Rotation2d minPivotAngle = new Rotation2d(-35.0);
  public static final Rotation2d maxPivotAngle = new Rotation2d(90.0);

  // Tunable numbers

  // Profile Tuning
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Extender/Pivot/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Extender/Pivot/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Extender/Pivot/kS");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Extender/Pivot/kG");
  private static final LoggedTunableNumber maxPivotVelocityDegPerSec =
      new LoggedTunableNumber("Extender/Pivot/MaxVelocityDegreesPerSec", 0.0);
  private static final LoggedTunableNumber maxPivotAccelerationDegPerSec2 =
      new LoggedTunableNumber("Extender/Pivot/MaxAccelerationDegreesPerSec2", 0.0);
  private static final LoggedTunableNumber algaePivotMaxVelocityDegPerSec =
      new LoggedTunableNumber("Extender/Pivot/AlgaeMaxVelocityDegreesPerSec", 0.0);
  private static final LoggedTunableNumber algaePivotMaxAccelerationDegPerSec2 =
      new LoggedTunableNumber("Extender/Pivot/AlgaeMaxAccelerationDegreesPerSec2", 0.0);
  public static final LoggedTunableNumber pivotTolerance =
      new LoggedTunableNumber("Extender/Pivot/Tolerance", 0.4);

  // Characterization Tuning
  private static final LoggedTunableNumber staticPivotCharacterizationVelocityThresh =
      new LoggedTunableNumber("Extender/Characterization/StaticVelocityThresh", 0.0);
  private static final LoggedTunableNumber staticPivotCharacterizationRampRate =
      new LoggedTunableNumber("Extender/Characterization/StaticRampRate", 0.0);

  // Gripper Tuning
  private static final LoggedTunableNumber algaeVelocityThresh =
      new LoggedTunableNumber("Extender/Gripper/AlgaeVelocityThreshold", 0.0);
  public static final LoggedTunableNumber gripperHoldVolts =
      new LoggedTunableNumber("Extender/Gripper/HoldVolts", 2.0);
  public static final LoggedTunableNumber gripperIntakeVolts =
      new LoggedTunableNumber("Extender/Gripper/IntakeVolts", 2.0);
  public static final LoggedTunableNumber gripperEjectVolts =
      new LoggedTunableNumber("Extender/Gripper/EjectVolts", -2.0);
  public static final LoggedTunableNumber gripperNetEjectVolts =
      new LoggedTunableNumber("Extender/Gripper/L1EjectVolts", -2.0);
  public static final LoggedTunableNumber gripperCurrentLimit =
      new LoggedTunableNumber("Extender/Gripper/CurrentLimit", 50.0);

  // Homing Tuning
  public static final LoggedTunableNumber homingPivotTimeSecs =
      new LoggedTunableNumber("Extender/Homing/TimeSecs", 0.2);
  public static final LoggedTunableNumber homingPivotVolts =
      new LoggedTunableNumber("Extender/Homing/Volts", 1.5);
  public static final LoggedTunableNumber homingPivotVelocityThresh =
      new LoggedTunableNumber("Extender/Homing/VelocityThreshold", 0.6);

  static {
    switch (Constants.getRobot()) {
      case COMPBOT, DEVBOT -> {
        kP.initDefault(0.0);
        kD.initDefault(0.0);
        kS.initDefault(0.0);
        kG.initDefault(0.0);
      }
      case SIMBOT -> {
        kP.initDefault(4000);
        kD.initDefault(1000);
        kS.initDefault(1.2);
        kG.initDefault(0.0);
      }
    }
  }

  public enum GripperGoal {
    IDLE,
    GRIP,
    EJECT,
    NET_EJECT
  }

  // Hardware
  private final ExtenderIO extenderIO;
  private final ExtenderIOInputsAutoLogged extenderInputs = new ExtenderIOInputsAutoLogged();
  private final RollerSystemIO gripperIO;
  private final RollerSystemIOInputsAutoLogged gripperInputs = new RollerSystemIOInputsAutoLogged();

  @AutoLogOutput(key = "Extender/Pivot/BrakeModeEnabled")
  private boolean pivotBrakeModeEnabled = true;

  @AutoLogOutput(key = "Extender/Gripper/BrakeModeEnabled")
  private boolean gripperBrakeModeEnabled = true;

  // Overrides
  private BooleanSupplier coastOverride = () -> false;
  private BooleanSupplier disableOverride = () -> false;
  private BooleanSupplier disableGamePieceDetectionOverride = () -> false;

  private TrapezoidProfile pivotProfile;
  private TrapezoidProfile algaeProfile;
  @Getter private State pivotSetpoint = new State();
  private DoubleSupplier pivotGoal = () -> 0.0;
  private boolean stopProfile = false;
  @Getter private boolean shouldEStop = false;
  @Getter private boolean isEStopped = false;
  @Getter private boolean isIntaking = false;

  @AutoLogOutput(key = "Extender/Pivot/Profile/PivotAtGoal")
  private boolean pivotAtGoal = false;

  @Setter private double gripperVolts = 0.0;
  @AutoLogOutput @Setter private GripperGoal gripperGoal = GripperGoal.IDLE;

  @Setter private boolean hasAlgae = false;
  @Setter private boolean shouldManipulateAlgae = false;

  private static final double algaeDebounceTime = 0.5;
  private Debouncer algaeDebouncer = new Debouncer(algaeDebounceTime, DebounceType.kRising);
  private Debouncer toleranceDebouncer = new Debouncer(0.25, DebounceType.kRising);
  private Debouncer homingDebouncer = new Debouncer(0.25, DebounceType.kRising);

  @AutoLogOutput private Rotation2d homingOffset = Rotation2d.kZero;

  // Disconnected Alerts
  private final Alert pivotMotorDisconnectedAlert =
      new Alert("Extender Pivot Motor Disconnected!", Alert.AlertType.kWarning);
  private final Alert gripperMotorDisconnectedAlert =
      new Alert("Extender Gripper Motor Disconnected!", Alert.AlertType.kWarning);

  public Extender(ExtenderIO extenderIO, RollerSystemIO gripperIO) {
    this.extenderIO = extenderIO;
    this.gripperIO = gripperIO;

    pivotProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(maxPivotVelocityDegPerSec.get()),
                Units.degreesToRadians(maxPivotAccelerationDegPerSec2.get())));
  }

  public void periodic() {
    extenderIO.updateInputs(extenderInputs);
    Logger.processInputs("Extender/PivotHardwareReadOuts", extenderInputs);
    gripperIO.updateInputs(gripperInputs);
    Logger.processInputs("Extender/GripperHardwareReadOuts", gripperInputs);

    pivotMotorDisconnectedAlert.set(
        !extenderInputs.data.motorConnected()
            && Constants.getRobot() == RobotType.COMPBOT
            && !Robot.isJITing());
    gripperMotorDisconnectedAlert.set(
        !gripperInputs.data.connected()
            && Constants.getRobot() == RobotType.COMPBOT
            && !Robot.isJITing());

    // Update Tunable numbers
    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      extenderIO.setPID(kP.get(), 0.0, kD.get());
    }
    if (maxPivotVelocityDegPerSec.hasChanged(hashCode())
        || maxPivotAccelerationDegPerSec2.hasChanged(hashCode())) {
      pivotProfile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  Units.degreesToRadians(maxPivotVelocityDegPerSec.get()),
                  Units.degreesToRadians(maxPivotAccelerationDegPerSec2.get())));
    }
    if (algaePivotMaxVelocityDegPerSec.hasChanged(hashCode())
        || algaePivotMaxAccelerationDegPerSec2.hasChanged(hashCode())) {
      algaeProfile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  Units.degreesToRadians(algaePivotMaxVelocityDegPerSec.get()),
                  Units.degreesToRadians(algaePivotMaxAccelerationDegPerSec2.get())));
    }

    // Set coast mode
    setPivotBrakeMode(!coastOverride.getAsBoolean());
    setGripperBrakeMode(!coastOverride.getAsBoolean());

    // Run Profile
    final boolean shouldRunProfile =
        !stopProfile
            && !coastOverride.getAsBoolean()
            && !disableOverride.getAsBoolean()
            && !isEStopped
            && DriverStation.isEnabled();
    Logger.recordOutput("Extender/Pivot/RunningProfile", shouldRunProfile);

    // Check if out of tolerance
    boolean outOfTolerance =
        Math.abs(getPivotAngle().getRadians() - pivotSetpoint.position) > pivotTolerance.get();

    Logger.recordOutput("Extender/Pivot/OutOfTolerance", pivotTolerance.get());

    shouldEStop = false; // toleranceDebouncer.calculate(outOfTolerance && shouldRunProfile);

    Logger.recordOutput("Extender/Pivot/ShouldEstop", shouldEStop);

    if (shouldRunProfile) {
      // Clamp goal
      var pivotGoalState =
          new State(
              MathUtil.clamp(
                  pivotGoal.getAsDouble(), minPivotAngle.getRadians(), maxPivotAngle.getRadians()),
              0.0);
      pivotSetpoint =
          (hasAlgae() ? algaeProfile : pivotProfile)
              .calculate(Constants.loopPeriodSecs, pivotSetpoint, pivotGoalState);
      extenderIO.runPosition(
          Rotation2d.fromRadians(
              pivotSetpoint.position - maxPivotAngle.getRadians() + homingOffset.getRadians()),
          kS.get() * Math.signum(pivotSetpoint.velocity) // Magnitude irrelevant
              + kG.get() * getPivotAngle().getCos());
      // Check at goal
      pivotAtGoal =
          EqualsUtil.epsilonEquals(pivotSetpoint.position, pivotGoalState.position)
              && EqualsUtil.epsilonEquals(pivotSetpoint.velocity, 0.0);

      // Log State
      Logger.recordOutput("Extender/Pivot/Profile/SetpointAngle", pivotSetpoint.position);
      Logger.recordOutput("Extender/Pivot/Profile/SetpointVelocity", pivotSetpoint.velocity);
      Logger.recordOutput("Extender/Pivot/Profile/GoalAngleRad", pivotGoalState.position);
    } else {
      // Reset position
      pivotSetpoint = new State(getPivotAngle().getRadians(), 0.0);

      // Clear logs
      Logger.recordOutput("Extender/Pivot/Profile/SetpointAngle", 0.0);
      Logger.recordOutput("Extender/Pivot/Profile/SetpointVelocity", 0.0);
      Logger.recordOutput("Extender/Pivot/Profile/GoalAngleRad", 0.0);
    }

    // Run gripper
    if (!isEStopped) {
      switch (gripperGoal) {
        case IDLE -> {
          gripperIO.stop();
          if (shouldManipulateAlgae && !hasAlgae) {
            setGripperGoal(GripperGoal.GRIP);
          }
        }
        case GRIP -> {
          if (hasAlgae) {
            gripperIO.runVolts(gripperHoldVolts.get());
          } else {
            if (shouldManipulateAlgae) {
              gripperIO.runVolts(gripperIntakeVolts.get());
            } else {
              setGripperGoal(GripperGoal.IDLE);
            }
          }
        }
        case EJECT -> {
          if (hasAlgae) {
            gripperIO.runVolts(gripperEjectVolts.get());
          } else {
            setGripperGoal(GripperGoal.IDLE); // Might need to add a timer before switching to IDLE
          }
        }
        case NET_EJECT -> {
          if (hasAlgae) {
            gripperIO.runVolts(gripperNetEjectVolts.get());
          } else {
            setGripperGoal(GripperGoal.IDLE); // Might need to add a timer before switching to IDLE
          }
        }
      }
    } else {
      extenderIO.stop();
      gripperIO.stop();
    }

    // Check algae state
    if (Constants.getRobot() != Constants.RobotType.SIMBOT) {
      if (Math.abs(gripperInputs.data.torqueCurrentAmps()) >= 5.0) {
        hasAlgae =
            algaeDebouncer.calculate(
                Math.abs(gripperInputs.data.velocityRadsPerSec()) <= algaeVelocityThresh.get());
      } else {
        algaeDebouncer.calculate(hasAlgae);
      }
    }

    // Display hasAlgae
    SmartDashboard.putBoolean("Has Algae", hasAlgae());

    // Log state
    Logger.recordOutput("Extender/Pivot/CoastOverride", coastOverride.getAsBoolean());
    Logger.recordOutput("Extender/Pivot/DisableOverride", disableOverride.getAsBoolean());

    // Record cycle time
    LoggedTracer.record("Extender");
  }

  public void setPivotGoal(Supplier<Rotation2d> pivotGoal) {
    this.pivotGoal =
        () ->
            MathUtil.inputModulus(
                pivotGoal.get().getRadians(), -3.0 * Math.PI / 2.0, Math.PI / 2.0);
    pivotAtGoal = false;
  }

  public double getGoal() {
    return pivotGoal.getAsDouble();
  }

  @AutoLogOutput
  public boolean hasAlgae() {
    return hasAlgae && !disableGamePieceDetectionOverride.getAsBoolean();
  }

  @AutoLogOutput(key = "Extender/Pivot/MeasuredAngleR2d")
  public Rotation2d getPivotAngle() {
    return extenderInputs.data.motorPosition();
  }

  @AutoLogOutput(key = "Extender/Pivot/MeasuredAngleDeg")
  public double getPivotAngleDeg() {
    return extenderInputs.data.motorPosition().getDegrees();
  }

  @AutoLogOutput(key = "Extender/Pivot/MeasuredAlteredAngleDeg")
  public double getPivotAngleAlteredDeg() {
    return extenderInputs.data.motorPosition().getDegrees() + homingOffset.getDegrees();
  }

  public void resetHasAlgae() {
    hasAlgae = false;
    algaeDebouncer = new Debouncer(algaeDebounceTime, DebounceType.kRising);
    algaeDebouncer.calculate(false);
  }

  public void setOverrides(
      BooleanSupplier coastOverride,
      BooleanSupplier disableOverride,
      BooleanSupplier disableGamePieceDetectionOverride) {
    this.coastOverride = coastOverride;
    this.disableOverride = disableOverride;
    this.disableGamePieceDetectionOverride = disableGamePieceDetectionOverride;
  }

  public Command homingSequence() {
    return Commands.startRun(
            () -> {
              stopProfile = true;
              homingDebouncer = new Debouncer(homingPivotTimeSecs.get(), DebounceType.kRising);
              homingDebouncer.calculate(false);
            },
            () -> {
              if (disableOverride.getAsBoolean() || coastOverride.getAsBoolean()) return;
              extenderIO.runVolts(homingPivotVolts.get());
            })
        .raceWith(
            Commands.runOnce(() -> {})
                .andThen(
                    Commands.waitUntil(
                        () ->
                            homingDebouncer.calculate(
                                Math.abs(extenderInputs.data.motorVelocityRadPerSec())
                                    <= homingPivotVelocityThresh.get()))))
        .andThen(
            () -> {
              homingOffset = extenderInputs.data.motorPosition();
            })
        .finallyDo(
            () -> {
              stopProfile = false;
            });
  }

  // Default Pivot Methods

  private void setPivotBrakeMode(boolean enabled) {
    if (pivotBrakeModeEnabled == enabled) return;
    pivotBrakeModeEnabled = enabled;
    extenderIO.setBrakeMode(enabled);
  }

  public void runPivotVolts(double volt) {
    extenderIO.runVolts(volt);
  }

  public void stopPivot() {
    extenderIO.stop();
  }

  // Default Gripper Methods

  private void setGripperBrakeMode(boolean enabled) {
    if (gripperBrakeModeEnabled == enabled) return;
    gripperBrakeModeEnabled = enabled;
    extenderIO.setBrakeMode(enabled);
  }

  public void runGripperVolts(double volt) {
    extenderIO.runVolts(volt);
  }

  public void stopGripper() {
    extenderIO.stop();
  }

  // Characterization Methods
  public Command staticCharacterization() {
    final StaticCharacterizationState state = new StaticCharacterizationState();
    Timer timer = new Timer();
    return Commands.startRun(
            () -> {
              stopProfile = true;
              timer.restart();
            },
            () -> {
              state.characterizationOutput =
                  staticPivotCharacterizationRampRate.get() * timer.get();
              extenderIO.runOpenLoop(state.characterizationOutput);
              Logger.recordOutput(
                  "Extender/Pivot/StaticCharacterizationOutput", state.characterizationOutput);
            })
        .until(
            () ->
                extenderInputs.data.motorVelocityRadPerSec()
                    >= staticPivotCharacterizationVelocityThresh.get())
        .andThen(extenderIO::stop)
        .andThen(Commands.idle())
        .finallyDo(
            () -> {
              stopProfile = false;
              timer.stop();
              Logger.recordOutput(
                  "Extender/Pivot/CharacterizationOutput", state.characterizationOutput);
            });
  }

  private static class StaticCharacterizationState {
    public double characterizationOutput = 0.0;
  }
}
