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
  public static final Rotation2d minAngle = new Rotation2d(0.0);
  public static final Rotation2d maxAngle = new Rotation2d(0.0);

  // Tunable numbers
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Beak/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Beak/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Beak/kS");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Beak/kG");
  private static final LoggedTunableNumber maxVelocityDegPerSec =
      new LoggedTunableNumber("Beak/MaxVelocityDegreesPerSec", 0.0);
  private static final LoggedTunableNumber maxAccelerationDegPerSec2 =
      new LoggedTunableNumber("Beak/MaxAccelerationDegreesPerSec2", 0.0);
  private static final LoggedTunableNumber coralMaxVelocityDegPerSec =
      new LoggedTunableNumber("Beak/CoralMaxVelocityDegreesPerSec", 0.0);
  private static final LoggedTunableNumber coralMaxAccelerationDegPerSec2 =
      new LoggedTunableNumber("Beak/CoralMaxAccelerationDegreesPerSec2", 0.0);
  private static final LoggedTunableNumber staticCharacterizationVelocityThresh =
      new LoggedTunableNumber("Beak/StaticCharacterizationVelocityThresh", 0.0);
  private static final LoggedTunableNumber staticCharacterizationRampRate =
      new LoggedTunableNumber("Beak/StaticCharacterizationRampRate", 0.0);
  private static final LoggedTunableNumber coralVelocityThresh =
      new LoggedTunableNumber("Beak/CoralVelocityThreshold", 0.0);
  public static final LoggedTunableNumber rollerHoldVolts =
      new LoggedTunableNumber("Beak/RollerHoldVolts", 0.0);
  public static final LoggedTunableNumber rollerIntakeVolts =
      new LoggedTunableNumber("Beak/RollerIntakeVolts", 0.0);
  public static final LoggedTunableNumber rollerEjectVolts =
      new LoggedTunableNumber("Beak/RollerEjectVolts", 0.0);
  public static final LoggedTunableNumber rollerL1EjectVolts =
      new LoggedTunableNumber("Beak/RollerL1EjectVolts", 0.0);
  public static final LoggedTunableNumber rollerCurrentLimit =
      new LoggedTunableNumber("Beak/RollerCurrentLimit", 0.0);
  public static final LoggedTunableNumber tolerance =
      new LoggedTunableNumber("Beak/Tolerance", 0.0);
  public static final LoggedTunableNumber intakeReverseVolts =
      new LoggedTunableNumber("Beak/IntakeReverseVolts", 0.0);
  public static final LoggedTunableNumber intakeReverseTime =
      new LoggedTunableNumber("Beak/IntakeReverseTime", 0.0);
  public static final LoggedTunableNumber homingTimeSecs =
      new LoggedTunableNumber("Beak/HomingTimeSecs", 0.0);
  public static final LoggedTunableNumber homingVolts =
      new LoggedTunableNumber("Beak/HomingVolts", 0.0);
  public static final LoggedTunableNumber homingVelocityThresh =
      new LoggedTunableNumber("Beak/HomingVelocityThreshold", 0.0);

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

  @AutoLogOutput(key = "Beak/BrakeModeEnabled")
  private boolean brakeModeEnabled = true;

  // Overrides
  private BooleanSupplier coastOverride = () -> false;
  private BooleanSupplier disableOverride = () -> false;
  private BooleanSupplier disableGamePieceDetectionOverride = () -> false;

  private TrapezoidProfile profile;
  private TrapezoidProfile coralProfile;
  @Getter private State setpoint = new State();
  private DoubleSupplier goal = () -> 0.0;
  private boolean stopProfile = false;
  @Getter private boolean shouldEStop = false;
  @Getter private boolean isEStopped = false;
  @Getter private boolean isIntaking = false;
  private final Timer intakingReverseTimer = new Timer();

  @AutoLogOutput(key = "Beak/Profile/AtGoal")
  private boolean atGoal = false;

  @Setter private double rollerVolts = 0.0;
  @AutoLogOutput @Setter private RollerGoal rollerGoal = RollerGoal.IDLE;

  @Setter private boolean hasCoral = false;
  @Getter private boolean doNotStopIntaking = false;

  private static final double coralDebounceTime = 0.5;
  private Debouncer coralDebouncer = new Debouncer(coralDebounceTime, DebounceType.kRising);
  private Debouncer toleranceDebouncer = new Debouncer(0.25, DebounceType.kRising);
  private Debouncer homingDebouncer = new Debouncer(0.25, DebounceType.kRising);

  @AutoLogOutput private Rotation2d homingOffset = Rotation2d.kZero;

  // Disconnected Alerts
  private final Alert pivotMotorDisconnectedAlert =
      new Alert("Beak Pivot Motor Disconnected!", Alert.AlertType.kWarning);
  private final Alert pivotEncoderDisconnectedAlert =
      new Alert("Beak Pivot Encoder Disconnected!", Alert.AlertType.kWarning);
  private final Alert rollerMotorDisconnectedAlert =
      new Alert("Beak Roller Motor Disconnected!", Alert.AlertType.kWarning);

  private boolean lastCoralButtonPressed = false;

  public Beak(BeakIO beakIO, RollerSystemIO rollerIO) {
    this.beakIO = beakIO;
    this.rollerIO = rollerIO;

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(maxVelocityDegPerSec.get()),
                Units.degreesToRadians(maxAccelerationDegPerSec2.get())));
    intakingReverseTimer.start();
  }

  public void periodic() {
    beakIO.updateInputs(beakInputs);
    Logger.processInputs("Beak/Pivot", beakInputs);
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs("Beak/Roller", rollerInputs);

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
    if (maxVelocityDegPerSec.hasChanged(hashCode())
        || maxAccelerationDegPerSec2.hasChanged(hashCode())) {
      profile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  Units.degreesToRadians(maxVelocityDegPerSec.get()),
                  Units.degreesToRadians(maxAccelerationDegPerSec2.get())));
    }
    if (coralMaxVelocityDegPerSec.hasChanged(hashCode())
        || coralMaxAccelerationDegPerSec2.hasChanged(hashCode())) {
      coralProfile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  Units.degreesToRadians(coralMaxVelocityDegPerSec.get()),
                  Units.degreesToRadians(coralMaxAccelerationDegPerSec2.get())));
    }
    if (rollerCurrentLimit.hasChanged(hashCode())) {
      rollerIO.setCurrentLimit(rollerCurrentLimit.get());
    }

    // Set coast mode
    setBrakeMode(!coastOverride.getAsBoolean());

    // Run Profile
    final boolean shouldRunProfile =
        !stopProfile
            && !coastOverride.getAsBoolean()
            && !disableOverride.getAsBoolean()
            && !isEStopped
            && DriverStation.isEnabled();
    Logger.recordOutput("Beak/RunningProfile", shouldRunProfile);

    // Check if out of tolerance
    boolean outOfTolerance =
        Math.abs(getPivotAngle().getRadians() - setpoint.position) > tolerance.get();
    shouldEStop = toleranceDebouncer.calculate(outOfTolerance && shouldRunProfile);
    if (shouldRunProfile) {
      // Clamp goal
      var goalState =
          new State(
              MathUtil.clamp(goal.getAsDouble(), minAngle.getRadians(), maxAngle.getRadians()),
              0.0);
      setpoint =
          (hasCoral() ? coralProfile : profile)
              .calculate(Constants.loopPeriodSecs, setpoint, goalState);
      beakIO.runPosition(
          Rotation2d.fromRadians(
              setpoint.position - maxAngle.getRadians() + homingOffset.getRadians()),
          kS.get() * Math.signum(setpoint.velocity) // Magnitude irrelevant
              + kG.get() * getPivotAngle().getCos());
      // Check at goal
      atGoal =
          EqualsUtil.epsilonEquals(setpoint.position, goalState.position)
              && EqualsUtil.epsilonEquals(setpoint.velocity, 0.0);

      // Log State
      Logger.recordOutput("Beak/Profile/SetpointAngle", setpoint.position);
      Logger.recordOutput("Beak/Profile/SetpointVelocity", setpoint.velocity);
      Logger.recordOutput("Beak/Profile/GoalAngleRad", goalState.position);
    } else {
      // Reset position
      setpoint = new State(getPivotAngle().getRadians(), 0.0);

      // Clear logs
      Logger.recordOutput("Beak/Profile/SetpointAngle", 0.0);
      Logger.recordOutput("Beak/Profile/SetpointVelocity", 0.0);
      Logger.recordOutput("Beak/Profile/GoalAngleRad", 0.0);
    }

    // Run Roller
    if (!isEStopped) {
      double intakeVolts = rollerVolts;
      if (isIntaking && !hasCoral) {
        intakingReverseTimer.restart();
      } else if (intakingReverseTimer.get() < intakeReverseTime.get()) {
        intakeVolts = intakeReverseVolts.get();
      } else if (isIntaking) {
        intakeVolts = 0.0;
      }
      rollerIO.runVolts(intakeVolts);
      switch (rollerGoal) {
        case IDLE -> rollerIO.stop();
        case GRIP -> {
          if (hasCoral) {
            rollerIO.runVolts(rollerHoldVolts.get());
          } else {
            rollerIO.runVolts(rollerIntakeVolts.get());
          }
        }
        case EJECT -> rollerIO.runVolts(rollerEjectVolts.get());
        case L1_EJECT -> rollerIO.runVolts(rollerL1EjectVolts.get());
      }
    } else {
      beakIO.stop();
      rollerIO.stop();
    }

    /* Add Code HERE ------------------------------------------------------------- */

    // Check Coral state

    /* Add Code HERE ------------------------------------------------------------- */

    // Display hasCoral
    SmartDashboard.putBoolean("Has Coral", hasCoral());

    // Log state
    Logger.recordOutput("Beak/CoastOverride", coastOverride.getAsBoolean());
    Logger.recordOutput("Beak/DisableOverride", disableOverride.getAsBoolean());

    // Record cycle time
    LoggedTracer.record("Beak");
  }

  public void setGoal(Supplier<Rotation2d> goal) {
    this.goal =
        () -> MathUtil.inputModulus(goal.get().getRadians(), -3.0 * Math.PI / 2.0, Math.PI / 2.0);
    atGoal = false;
  }

  public double atGoal() {
    return goal.getAsDouble();
  }

  @AutoLogOutput
  public boolean hasCoral() {
    return hasCoral && !disableGamePieceDetectionOverride.getAsBoolean();
  }

  @AutoLogOutput(key = "Beak/MeasuredAngle")
  public Rotation2d getPivotAngle() {
    return beakInputs.data.positionRad().plus(maxAngle).minus(homingOffset);
  }

  public void resetHasCoral() {
    hasCoral = false;
    coralDebouncer = new Debouncer(coralDebounceTime, DebounceType.kRising);
    coralDebouncer.calculate(false);
  }

  public void setOverrides(
      BooleanSupplier coastOverride,
      BooleanSupplier disableOverride,
      BooleanSupplier disableGamePieceDetectionOverride) {
    this.coastOverride = coastOverride;
    this.disableOverride = disableOverride;
    this.disableGamePieceDetectionOverride = disableGamePieceDetectionOverride;
  }

  private void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    beakIO.setBrakeMode(enabled);
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
              state.characterizationOutput = staticCharacterizationRampRate.get() * timer.get();
              beakIO.runOpenLoop(state.characterizationOutput);
              Logger.recordOutput(
                  "Beak/StaticCharacterizationOutput", state.characterizationOutput);
            })
        .until(
            () -> beakInputs.data.velocityRadPerSec() >= staticCharacterizationVelocityThresh.get())
        .andThen(beakIO::stop)
        .andThen(Commands.idle())
        .finallyDo(
            () -> {
              stopProfile = false;
              timer.stop();
              Logger.recordOutput("Beak/CharacterizationOutput", state.characterizationOutput);
            });
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
                                Math.abs(beakInputs.data.velocityRadPerSec())
                                    <= homingVelocityThresh.get()))))
        .andThen(
            () -> {
              homingOffset = beakInputs.data.positionRad();
            })
        .finallyDo(
            () -> {
              stopProfile = false;
            });
  }

  public void runVolts(double volt) {
    beakIO.runVolts(volt);
  }

  private static class StaticCharacterizationState {
    public double characterizationOutput = 0.0;
  }
}
