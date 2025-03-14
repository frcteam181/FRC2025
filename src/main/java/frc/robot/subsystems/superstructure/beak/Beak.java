package frc.robot.subsystems.superstructure.beak;

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

public class Beak {
  public static final Rotation2d minPivotAngle = Rotation2d.fromDegrees(-180.0);
  public static final Rotation2d maxPivotAngle = Rotation2d.fromDegrees(0.0);

  // Pivot Tuner
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Beak/Pivot/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Beak/Pivot/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Beak/Pivot/kS");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Beak/Pivot/kG");
  private static final LoggedTunableNumber maxPivotVelocityDegPerSec =
      new LoggedTunableNumber("Beak/Pivot/MaxVelocityDegreesPerSec", 0.0);
  private static final LoggedTunableNumber maxPivotAccelerationDegPerSec2 =
      new LoggedTunableNumber("Beak/Pivot/MaxAccelerationDegreesPerSec2", 0.0);
  private static final LoggedTunableNumber coralMaxPivotVelocityDegPerSec =
      new LoggedTunableNumber("Beak/Pivot/CoralMaxVelocityDegreesPerSec", 0.0);
  private static final LoggedTunableNumber coralMaxPivotAccelerationDegPerSec2 =
      new LoggedTunableNumber("Beak/Pivot/CoralMaxAccelerationDegreesPerSec2", 0.0);
  public static final LoggedTunableNumber pivotTolerance =
      new LoggedTunableNumber("Beak/Tolerance", 0.0);

  // Characterization Tuner
  private static final LoggedTunableNumber staticPivotCharacterizationVelocityThresh =
      new LoggedTunableNumber("Beak/Characterization/StaticVelocityThresh", 0.0);
  private static final LoggedTunableNumber staticPivotCharacterizationRampRate =
      new LoggedTunableNumber("Beak/Characterization/StaticRampRate", 0.0);

  // Roller Tuner
  public static final LoggedTunableNumber rollerIntakeVolts =
      new LoggedTunableNumber("Beak/Roller/IntakeVolts", 4.0);
  public static final LoggedTunableNumber rollerHoldVolts =
      new LoggedTunableNumber("Beak/Roller/HoldVolts", 0.1);
  private static final LoggedTunableNumber coralVelocityThresh =
      new LoggedTunableNumber("Beak/Roller/CoralVelocityThreshold", 0.2);
  public static final LoggedTunableNumber rollerEjectVolts =
      new LoggedTunableNumber("Beak/Roller/EjectVolts", -3.0);
  public static final LoggedTunableNumber rollerL1EjectVolts =
      new LoggedTunableNumber("Beak/Roller/L1EjectVolts", -1.0);
  public static final LoggedTunableNumber rollerCurrentLimit =
      new LoggedTunableNumber("Beak/Roller/CurrentLimit", 50.0);
  public static final LoggedTunableNumber ejectTime =
      new LoggedTunableNumber("Beak/Roller/EjectTime", 0.2);

  private final Timer ejectTimer = new Timer();

  // Homing Tuner
  public static final LoggedTunableNumber homingTimeSecs =
      new LoggedTunableNumber("Beak/Homing/TimeSecs", 0.0);
  public static final LoggedTunableNumber homingVolts =
      new LoggedTunableNumber("Beak/Homing/Volts", 0.0);
  public static final LoggedTunableNumber homingVelocityThresh =
      new LoggedTunableNumber("Beak/Homing/VelocityThreshold", 0.0);

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

  public enum RollerGoal {
    IDLE,
    GRIP,
    EJECT,
    L1_EJECT
  }

  // Hardware
  private final BeakIO beakIO;
  private final PivotIOInputsAutoLogged beakInputs = new PivotIOInputsAutoLogged();
  private final RollerSystemIO rollerIO;
  private final RollerSystemIOInputsAutoLogged rollerInputs = new RollerSystemIOInputsAutoLogged();

  @AutoLogOutput(key = "Beak/Pivot/BrakeModeEnabled")
  private boolean pivotBrakeModeEnabled = true;

  @AutoLogOutput(key = "Beak/Roller/BrakeModeEnabled")
  private boolean rollerBrakeModeEnabled = true;

  // Overrides
  private BooleanSupplier coastOverride = () -> false;
  private BooleanSupplier disableOverride = () -> false;
  private BooleanSupplier disableGamePieceDetectionOverride = () -> false;

  private TrapezoidProfile pivotProfile;
  private TrapezoidProfile coralProfile;
  @Getter private State pivotSetpoint = new State();
  private DoubleSupplier pivotGoal = () -> 0.0;
  private boolean stopProfile = false;
  @Getter private boolean shouldEStop = false;
  @Getter private boolean isEStopped = false;
  @Getter private boolean isIntaking = false;

  @AutoLogOutput(key = "Beak/Pivot/Profile/AtGoal")
  private boolean pivotAtGoal = false;

  @Setter private double rollerVolts = 0.0;
  @AutoLogOutput @Setter private RollerGoal rollerGoal = RollerGoal.IDLE;

  @Setter private boolean hasCoral = false;
  @Setter private boolean shouldManipulateCoral = false;

  private static final double coralDebounceTime = 0.5;
  private Debouncer coralDebouncer = new Debouncer(coralDebounceTime, DebounceType.kRising);
  private Debouncer toleranceDebouncer = new Debouncer(0.25, DebounceType.kRising);
  private Debouncer homingDebouncer = new Debouncer(0.25, DebounceType.kRising);

  @AutoLogOutput private Rotation2d homingOffset = Rotation2d.kZero;

  // Disconnected Alerts
  private final Alert pivotMotorDisconnectedAlert =
      new Alert("Beak Pivot Motor Disconnected!", Alert.AlertType.kWarning);
  private final Alert rollerMotorDisconnectedAlert =
      new Alert("Beak Roller Motor Disconnected!", Alert.AlertType.kWarning);

  public Beak(BeakIO beakIO, RollerSystemIO rollerIO) {
    this.beakIO = beakIO;
    this.rollerIO = rollerIO;

    pivotProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(maxPivotVelocityDegPerSec.get()),
                Units.degreesToRadians(maxPivotAccelerationDegPerSec2.get())));
  }

  public void periodic() {
    beakIO.updateInputs(beakInputs);
    Logger.processInputs("Beak/PivotHardwareReadouts", beakInputs);
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs("Beak/RollerHardwareReadOuts", rollerInputs);

    pivotMotorDisconnectedAlert.set(
        !beakInputs.data.motorConnected()
            && Constants.getRobot() == RobotType.COMPBOT
            && !Robot.isJITing());
    rollerMotorDisconnectedAlert.set(
        !rollerInputs.data.connected()
            && Constants.getRobot() == RobotType.COMPBOT
            && !Robot.isJITing());

    // Update Tunable numbers
    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      beakIO.setPID(kP.get(), 0.0, kD.get());
    }
    if (maxPivotVelocityDegPerSec.hasChanged(hashCode())
        || maxPivotAccelerationDegPerSec2.hasChanged(hashCode())) {
      pivotProfile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  Units.degreesToRadians(maxPivotVelocityDegPerSec.get()),
                  Units.degreesToRadians(maxPivotAccelerationDegPerSec2.get())));
    }
    if (coralMaxPivotVelocityDegPerSec.hasChanged(hashCode())
        || coralMaxPivotAccelerationDegPerSec2.hasChanged(hashCode())) {
      coralProfile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  Units.degreesToRadians(coralMaxPivotVelocityDegPerSec.get()),
                  Units.degreesToRadians(coralMaxPivotAccelerationDegPerSec2.get())));
    }
    if (rollerCurrentLimit.hasChanged(hashCode())) {
      rollerIO.setCurrentLimit(rollerCurrentLimit.get());
    }

    // Set coast mode
    setPivotBrakeMode(!coastOverride.getAsBoolean());
    setRollerBrakeMode(!coastOverride.getAsBoolean());

    // Run Profile
    final boolean shouldRunProfile =
        !stopProfile
            && !coastOverride.getAsBoolean()
            && !disableOverride.getAsBoolean()
            && !isEStopped
            && DriverStation.isEnabled();
    Logger.recordOutput("Beak/Pivot/RunningProfile", shouldRunProfile);

    // Check if out of tolerance
    boolean outOfTolerance =
        Math.abs(getPivotAngle().getRadians() - pivotSetpoint.position) > pivotTolerance.get();

    Logger.recordOutput("Beak/Pivot/OutOfTolerance", pivotTolerance.get());

    shouldEStop = false; // toleranceDebouncer.calculate(outOfTolerance && shouldRunProfile);
    if (shouldRunProfile) {
      // Clamp goal
      var pivotGoalState =
          new State(
              MathUtil.clamp(
                  pivotGoal.getAsDouble(), minPivotAngle.getRadians(), maxPivotAngle.getRadians()),
              0.0);
      pivotSetpoint =
          (hasCoral() ? coralProfile : pivotProfile)
              .calculate(Constants.loopPeriodSecs, pivotSetpoint, pivotGoalState);
      beakIO.runPosition(
          Rotation2d.fromRadians(
              pivotSetpoint.position), // - maxPivotAngle.getRadians() + homingOffset.getRadians()),
          kS.get() * Math.signum(pivotSetpoint.velocity) // Magnitude irrelevant
              + kG.get() * getPivotAngle().getCos());
      // Check at goal
      pivotAtGoal =
          EqualsUtil.epsilonEquals(pivotSetpoint.position, pivotGoalState.position)
              && EqualsUtil.epsilonEquals(pivotSetpoint.velocity, 0.0);

      // Log State
      Logger.recordOutput("Beak/Pivot/Profile/SetpointAngle", pivotSetpoint.position);
      Logger.recordOutput("Beak/Pivot/Profile/SetpointVelocity", pivotSetpoint.velocity);
      Logger.recordOutput("Beak/Pivot/Profile/GoalAngleRad", pivotGoalState.position);

      // Log state in Deg
      Logger.recordOutput(
          "Beak/Pivot/Profile/SetpointAngleDeg", Math.toDegrees(pivotSetpoint.position));
      Logger.recordOutput(
          "Beak/Pivot/Profile/SetpointVelocityDeg", Math.toDegrees(pivotSetpoint.velocity));
      Logger.recordOutput(
          "Beak/Pivot/Profile/GoalAngleDeg", Math.toDegrees(pivotGoalState.position));
    } else {
      // Reset position
      pivotSetpoint = new State(getPivotAngle().getRadians(), 0.0);

      // Clear logs
      Logger.recordOutput("Beak/Pivot/Profile/SetpointAngle", 0.0);
      Logger.recordOutput("Beak/Pivot/Profile/SetpointVelocity", 0.0);
      Logger.recordOutput("Beak/Pivot/Profile/GoalAngleRad", 0.0);

      // Clear log in Deg
      Logger.recordOutput("Beak/Pivot/Profile/SetpointAngleDeg", 0.0);
      Logger.recordOutput("Beak/Pivot/Profile/SetpointVelocityDeg", 0.0);
      Logger.recordOutput("Beak/Pivot/Profile/GoalAngleDeg", 0.0);
    }

    // Run roller
    if (!isEStopped) {
      switch (rollerGoal) {
        case IDLE -> {
          rollerIO.stop();
          if (shouldManipulateCoral && !hasCoral) {
            setRollerGoal(RollerGoal.GRIP);
          }
        }
        case GRIP -> {
          if (hasCoral) {
            rollerIO.runVolts(rollerHoldVolts.get());
          } else {
            if (shouldManipulateCoral) {
              rollerIO.runVolts(rollerIntakeVolts.get());
            } else {
              setRollerGoal(RollerGoal.IDLE);
            }
          }
        }
        case EJECT -> {
          if (hasCoral) {
            rollerIO.runVolts(rollerEjectVolts.get());
            ejectTimer.start();
          } else {
            rollerIO.runVolts(rollerEjectVolts.get());
            if (ejectTimer.hasElapsed(ejectTime.get())) {
              ejectTimer.stop();
              ejectTimer.reset();
              setRollerGoal(RollerGoal.IDLE);
            }
          }
        }
        case L1_EJECT -> {
          if (hasCoral) {
            rollerIO.runVolts(rollerL1EjectVolts.get());
          } else {
            rollerIO.runVolts(rollerL1EjectVolts.get());
            if (ejectTimer.hasElapsed(ejectTime.get())) {
              ejectTimer.stop();
              ejectTimer.reset();
              setRollerGoal(RollerGoal.IDLE);
            }
          }
        }
      }
    } else {
      beakIO.stop();
      rollerIO.stop();
    }

    // Check coral state
    if (Constants.getRobot() != Constants.RobotType.SIMBOT) {
      if (Math.abs(rollerInputs.data.torqueCurrentAmps()) >= 5.0) {
        hasCoral =
            coralDebouncer.calculate(
                Math.abs(rollerInputs.data.velocityRadsPerSec()) <= coralVelocityThresh.get());
      } else {
        coralDebouncer.calculate(hasCoral);
      }
    }

    // Display hasCoral
    SmartDashboard.putBoolean("Has Coral", hasCoral());

    // Log state
    Logger.recordOutput("Beak/Pivot/CoastOverride", coastOverride.getAsBoolean());
    Logger.recordOutput("Beak/Pivot/DisableOverride", disableOverride.getAsBoolean());

    // Record cycle time
    LoggedTracer.record("Beak");
  }

  @AutoLogOutput(key = "Beak/Pivot/MeasuredAngleDeg")
  public double getMeasuredAngleDeg() {
    return beakInputs.data.motorPosition().getDegrees();
  }

  @AutoLogOutput(key = "Beak/Pivot/MeasuredVelocityDeg")
  public double getMeasuredVelocityDeg() {
    return Math.toDegrees(beakInputs.data.motorVelocityRadPerSec());
  }

  public void setGoal(Supplier<Rotation2d> pivotGoal) {
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
  public boolean hasCoral() {
    return hasCoral && !disableGamePieceDetectionOverride.getAsBoolean();
  }

  @AutoLogOutput(key = "Beak/Pivot/MeasuredAngle")
  public Rotation2d getPivotAngle() {
    return beakInputs.data.motorPosition(); // .plus(maxPivotAngle).minus(homingOffset);
  }

  public void resetHasCoral() {
    hasCoral = false;
    coralDebouncer = new Debouncer(coralDebounceTime, DebounceType.kRising);
    coralDebouncer.calculate(false);
  }

  public void startEjectTimer() {
    ejectTimer.start();
  }

  public void setOverrides(
      BooleanSupplier coastOverride,
      BooleanSupplier disableOverride,
      BooleanSupplier disableGamePieceDetectionOverride) {
    this.coastOverride = coastOverride;
    this.disableOverride = disableOverride;
    this.disableGamePieceDetectionOverride = disableGamePieceDetectionOverride;
  }

  private void setPivotBrakeMode(boolean enabled) {
    if (pivotBrakeModeEnabled == enabled) return;
    pivotBrakeModeEnabled = enabled;
    beakIO.setBrakeMode(enabled);
  }

  private void setRollerBrakeMode(boolean enabled) {
    if (rollerBrakeModeEnabled == enabled) return;
    rollerBrakeModeEnabled = enabled;
    rollerIO.setBrakeMode(enabled);
  }

  public Command homingSequence() {
    return Commands.startRun(
            () -> {
              stopProfile = true;
              homingDebouncer = new Debouncer(homingTimeSecs.get(), DebounceType.kRising);
              homingDebouncer.calculate(false);
            },
            () -> {
              if (disableOverride.getAsBoolean() || coastOverride.getAsBoolean()) return;
              beakIO.runVolts(homingVolts.get());
            })
        .raceWith(
            Commands.runOnce(() -> {})
                .andThen(
                    Commands.waitUntil(
                        () ->
                            homingDebouncer.calculate(
                                Math.abs(beakInputs.data.motorVelocityRadPerSec())
                                    <= homingVelocityThresh.get()))))
        .andThen(
            () -> {
              homingOffset = beakInputs.data.motorPosition();
            })
        .finallyDo(
            () -> {
              stopProfile = false;
            });
  }

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
              beakIO.runOpenLoop(state.characterizationOutput);
              Logger.recordOutput(
                  "Beak/StaticCharacterizationOutput", state.characterizationOutput);
            })
        .until(
            () ->
                beakInputs.data.motorVelocityRadPerSec()
                    >= staticPivotCharacterizationVelocityThresh.get())
        .andThen(beakIO::stop)
        .andThen(Commands.idle())
        .finallyDo(
            () -> {
              stopProfile = false;
              timer.stop();
              Logger.recordOutput("Beak/CharacterizationOutput", state.characterizationOutput);
            });
  }

  public void runPivotOpenLoop(double output) {
    beakIO.runVolts(output);
  }

  private static class StaticCharacterizationState {
    public double characterizationOutput = 0.0;
  }
}
