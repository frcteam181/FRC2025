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
  public static final Rotation2d minAngle = new Rotation2d(0.0);
  public static final Rotation2d maxAngle = new Rotation2d(0.0);

  // Tunable numbers
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Extender/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Extender/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Extender/kS");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Extender/kG");
  private static final LoggedTunableNumber maxVelocityDegPerSec =
      new LoggedTunableNumber("Extender/MaxVelocityDegreesPerSec", 0.0);
  private static final LoggedTunableNumber maxAccelerationDegPerSec2 =
      new LoggedTunableNumber("Extender/MaxAccelerationDegreesPerSec2", 0.0);
  private static final LoggedTunableNumber algaeMaxVelocityDegPerSec =
      new LoggedTunableNumber("Extender/AlgaeMaxVelocityDegreesPerSec", 0.0);
  private static final LoggedTunableNumber algaeMaxAccelerationDegPerSec2 =
      new LoggedTunableNumber("Extender/AlgaeMaxAccelerationDegreesPerSec2", 0.0);
  private static final LoggedTunableNumber staticCharacterizationVelocityThresh =
      new LoggedTunableNumber("Extender/StaticCharacterizationVelocityThresh", 0.0);
  private static final LoggedTunableNumber staticCharacterizationRampRate =
      new LoggedTunableNumber("Extender/StaticCharacterizationRampRate", 0.0);
  private static final LoggedTunableNumber algaeVelocityThresh =
      new LoggedTunableNumber("Extender/AlgaeVelocityThreshold", 0.0);
  public static final LoggedTunableNumber gripperHoldVolts =
      new LoggedTunableNumber("Extender/GripperHoldVolts", 2.0);
  public static final LoggedTunableNumber gripperIntakeVolts =
      new LoggedTunableNumber("Extender/GripperIntakeVolts", 2.0);
  public static final LoggedTunableNumber gripperEjectVolts =
      new LoggedTunableNumber("Extender/GripperEjectVolts", 0.0);
  public static final LoggedTunableNumber gripperL1EjectVolts =
      new LoggedTunableNumber("Extender/GripperL1EjectVolts", 0.0);
  public static final LoggedTunableNumber gripperCurrentLimit =
      new LoggedTunableNumber("Extender/GripperCurrentLimit", 0.0);
  public static final LoggedTunableNumber tolerance =
      new LoggedTunableNumber("Extender/Tolerance", 0.0);
  public static final LoggedTunableNumber intakeReverseVolts =
      new LoggedTunableNumber("Extender/IntakeReverseVolts", 0.0);
  public static final LoggedTunableNumber intakeReverseTime =
      new LoggedTunableNumber("Extender/IntakeReverseTime", 0.0);
  public static final LoggedTunableNumber homingTimeSecs =
      new LoggedTunableNumber("Extender/HomingTimeSecs", 0.0);
  public static final LoggedTunableNumber homingVolts =
      new LoggedTunableNumber("Extender/HomingVolts", 0.0);
  public static final LoggedTunableNumber homingVelocityThresh =
      new LoggedTunableNumber("Extender/HomingVelocityThreshold", 0.0);

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
  }

  // Hardware
  private final ExtenderIO extenderIO;
  private final ExtenderIOInputsAutoLogged extenderInputs = new ExtenderIOInputsAutoLogged();
  private final RollerSystemIO gripperIO;
  private final RollerSystemIOInputsAutoLogged gripperInputs = new RollerSystemIOInputsAutoLogged();

  @AutoLogOutput(key = "Extender/BrakeModeEnabled")
  private boolean brakeModeEnabled = true;

  // Overrides
  private BooleanSupplier coastOverride = () -> false;
  private BooleanSupplier disableOverride = () -> false;
  private BooleanSupplier disableGamePieceDetectionOverride = () -> false;

  private TrapezoidProfile profile;
  private TrapezoidProfile algaeProfile;
  @Getter private State setpoint = new State();
  private DoubleSupplier goal = () -> 0.0;
  private boolean stopProfile = false;
  @Getter private boolean shouldEStop = false;
  @Getter private boolean isEStopped = false;
  @Getter private boolean isIntaking = false;
  private final Timer intakingReverseTimer = new Timer();

  @AutoLogOutput(key = "Extender/Profile/AtGoal")
  private boolean atGoal = false;

  @Setter private double gripperVolts = 0.0;
  @AutoLogOutput @Setter private GripperGoal gripperGoal = GripperGoal.IDLE;

  @Setter private boolean hasAlgae = false;
  @Getter private boolean doNotStopIntaking = false;

  private static final double algaeDebounceTime = 0.5;
  private Debouncer algaeDebouncer = new Debouncer(algaeDebounceTime, DebounceType.kRising);
  private Debouncer toleranceDebouncer = new Debouncer(0.25, DebounceType.kRising);
  private Debouncer homingDebouncer = new Debouncer(0.25, DebounceType.kRising);

  //@AutoLogOutput private Rotation2d homingOffset = Rotation2d.kZero; We don't use this variable since we have absolute encoders

  // Disconnected Alerts
  private final Alert pivotMotorDisconnectedAlert =
      new Alert("Extender Motor Disconnected!", Alert.AlertType.kWarning);
  private final Alert pivotEncoderDisconnectedAlert =
      new Alert("Extender Encoder Disconnected!", Alert.AlertType.kWarning);
  private final Alert gripperMotorDisconnectedAlert =
      new Alert("Gripper Motor Disconnected!", Alert.AlertType.kWarning);

  private boolean lastAlgaeButtonPressed = false;

  public Extender(ExtenderIO extenderIO, RollerSystemIO gripperIO) {
    this.extenderIO = extenderIO;
    this.gripperIO = gripperIO;

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(maxVelocityDegPerSec.get()),
                Units.degreesToRadians(maxAccelerationDegPerSec2.get())));
    intakingReverseTimer.start();
  }

  public void periodic() {
    extenderIO.updateInputs(extenderInputs);
    Logger.processInputs("Extender/Pivot", extenderInputs);
    gripperIO.updateInputs(gripperInputs);
    Logger.processInputs("Extender/Gripper", gripperInputs);

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
    if (maxVelocityDegPerSec.hasChanged(hashCode())
        || maxAccelerationDegPerSec2.hasChanged(hashCode())) {
      profile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  Units.degreesToRadians(maxVelocityDegPerSec.get()),
                  Units.degreesToRadians(maxAccelerationDegPerSec2.get())));
    }
    if (algaeMaxVelocityDegPerSec.hasChanged(hashCode())
        || algaeMaxAccelerationDegPerSec2.hasChanged(hashCode())) {
      algaeProfile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  Units.degreesToRadians(algaeMaxVelocityDegPerSec.get()),
                  Units.degreesToRadians(algaeMaxAccelerationDegPerSec2.get())));
    }
    if (gripperCurrentLimit.hasChanged(hashCode())) {
      gripperIO.setCurrentLimit(gripperCurrentLimit.get());
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
    Logger.recordOutput("Extender/RunningProfile", shouldRunProfile);

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
          (hasAlgae() ? algaeProfile : profile)
              .calculate(Constants.loopPeriodSecs, setpoint, goalState);
      extenderIO.runPosition(
          Rotation2d.fromRadians(
              setpoint.position - maxAngle.getRadians() /**+ homingOffset.getRadians()*/),
          kS.get() * Math.signum(setpoint.velocity) // Magnitude irrelevant
              + kG.get() * getPivotAngle().getCos());
      // Check at goal
      atGoal =
          EqualsUtil.epsilonEquals(setpoint.position, goalState.position)
              && EqualsUtil.epsilonEquals(setpoint.velocity, 0.0);

      // Log State
      Logger.recordOutput("Extender/Profile/SetpointAngle", setpoint.position);
      Logger.recordOutput("Extender/Profile/SetpointVelocity", setpoint.velocity);
      Logger.recordOutput("Extender/Profile/GoalAngleRad", goalState.position);
    } else {
      // Reset position
      setpoint = new State(getPivotAngle().getRadians(), 0.0);

      // Clear logs
      Logger.recordOutput("Extender/Profile/SetpointAngle", 0.0);
      Logger.recordOutput("Extender/Profile/SetpointVelocity", 0.0);
      Logger.recordOutput("Extender/Profile/GoalAngleRad", 0.0);
    }

    // Run Gripper
    if (!isEStopped) {
      double intakeVolts = gripperVolts;
      if (isIntaking && !hasAlgae) {
        intakingReverseTimer.restart();
      } else if (intakingReverseTimer.get() < intakeReverseTime.get()) {
        intakeVolts = intakeReverseVolts.get();
      } else if (isIntaking) {
        intakeVolts = 0.0;
      }
    } else {
      extenderIO.stop();
      gripperIO.stop();
    }

    /* Add Code HERE ------------------------------------------------------------- */

    // Check Algae state

    /* Add Code HERE ------------------------------------------------------------- */

    // Display hasAlgae
    SmartDashboard.putBoolean("Has Algae", hasAlgae());

    // Log state
    Logger.recordOutput("Extender/CoastOverride", coastOverride.getAsBoolean());
    Logger.recordOutput("Extender/DisableOverride", disableOverride.getAsBoolean());

    // Record cycle time
    LoggedTracer.record("Extender");
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
  public boolean hasAlgae() {
    return hasAlgae && !disableGamePieceDetectionOverride.getAsBoolean();
  }

  @AutoLogOutput(key = "Extender/MeasuredAngle")
  public Rotation2d getPivotAngle() {
    return extenderInputs.data.position();
  }

  @AutoLogOutput(key = "Extender/MeasuredAngleDeg")
  public double getPivotAngleDeg() {
    return extenderInputs.data.position().getDegrees();
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

  private void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    extenderIO.setBrakeMode(enabled);
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
              extenderIO.runOpenLoop(state.characterizationOutput);
              Logger.recordOutput(
                  "Extender/StaticCharacterizationOutput", state.characterizationOutput);
            })
        .until(
            () ->
                extenderInputs.data.velocityRadPerSec()
                    >= staticCharacterizationVelocityThresh.get())
        .andThen(extenderIO::stop)
        .andThen(Commands.idle())
        .finallyDo(
            () -> {
              stopProfile = false;
              timer.stop();
              Logger.recordOutput("Extender/CharacterizationOutput", state.characterizationOutput);
            });
  }

  // public Command homingSequence() {
  //   return Commands.startRun(
  //           () -> {
  //             stopProfile = true;
  //             homingDebouncer = new Debouncer(homingTimeSecs.get(), DebounceType.kRising);
  //             homingDebouncer.calculate(false);
  //           },
  //           () -> {
  //             if (disableOverride.getAsBoolean() || coastOverride.getAsBoolean()) return;
  //             extenderIO.runVolts(homingVolts.get());
  //           })
  //       .raceWith(
  //           Commands.runOnce(() -> {})
  //               .andThen(
  //                   Commands.waitUntil(
  //                       () ->
  //                           homingDebouncer.calculate(
  //                               Math.abs(extenderInputs.data.velocityRadPerSec())
  //                                   <= homingVelocityThresh.get()))))
  //       .andThen(
  //           () -> {
  //             homingOffset = extenderInputs.data.position();
  //           })
  //       .finallyDo(
  //           () -> {
  //             stopProfile = false;
  //           });
  // }

  public void runVolts(double volt) {
    extenderIO.runVolts(volt);
  }

  public void runGrip(double volt) {
    gripperIO.runVolts(volt);
  }

  public void stopGrip() {
    gripperIO.stop();
  }

  private static class StaticCharacterizationState {
    public double characterizationOutput = 0.0;
  }
}
