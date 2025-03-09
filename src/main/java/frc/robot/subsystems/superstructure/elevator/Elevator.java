package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.superstructure.SuperstructureConstants.ElevatorConstants;
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

public class Elevator {
  public static final double sprocketRadius =
      Units.inchesToMeters((1.757 / 2.0)) * 2.0; // Multiplying by 2 because there are 2 stages
  private static final LoggedTunableNumber characterizationRampRate =
      new LoggedTunableNumber("Elevator/CharacterizationRampRate", 1.1);
  private static final LoggedTunableNumber characterizationUpVelocityThresh =
      new LoggedTunableNumber("Elevator/CharacterizationUpVelocityThresh", 0.7);
  private static final LoggedTunableNumber characterizationDownStartAmps =
      new LoggedTunableNumber("Elevator/CharacterizationDownStartAmps", 0.0);
  private static final LoggedTunableNumber characterizationDownVelocityThresh =
      new LoggedTunableNumber("Elevator/CharacterizationDownVelocityThresh", -0.1);

  // Tunable Numbers
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA");

  private static final LoggedTunableNumber maxVelocityMeterPerSec =
      new LoggedTunableNumber("Elevator/MaxVelocityMeterPerSec", 0.3); // 2.5
  private static final LoggedTunableNumber maxAccelerationMeterPerSec2 =
      new LoggedTunableNumber("Elevator/MaxAccelerationMeterPerSec2", 0.3); // 8.0

  private static final LoggedTunableNumber homingVolts =
      new LoggedTunableNumber("Elevator/HomingVoltage", -2.0);
  private static final LoggedTunableNumber homingTimeSecs =
      new LoggedTunableNumber("Elevator/HomingTimeSecs", 0.25);
  private static final LoggedTunableNumber homingVelocityThresh =
      new LoggedTunableNumber("Elevator/HomingVelocityThresh", 0.3); // 5.0
  private static final LoggedTunableNumber tolerance =
      new LoggedTunableNumber("Elevator/Tolerance", 0.5);

  private final ElevatorIO elevatorIO;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  // Alerts
  private final Alert leaderMotorDisconnectedAlert =
      new Alert("Elevator leader motor disconnected!", Alert.AlertType.kWarning);
  private final Alert followerMotorDisconnectedAlert =
      new Alert("Elevator follower motor disconnected!", Alert.AlertType.kWarning);
  private BooleanSupplier coastOverride = () -> false;
  private BooleanSupplier disabledOverride = () -> false;

  @AutoLogOutput private boolean brakeModeEnabled = true;

  private TrapezoidProfile profile;
  @Getter private State setpoint = new State();
  private Supplier<State> goal = State::new;
  private boolean stopProfile = false;
  @Getter private boolean shouldEStop = false;
  @Setter private boolean isEStopped = false;

  @AutoLogOutput(key = "Elevator/HomePositionRad")
  private double homePositionRad = 0.0;

  @AutoLogOutput @Getter private boolean homed = true;

  private Debouncer homingDebouncer = new Debouncer(homingTimeSecs.get());

  private Debouncer toleranceDebouncer = new Debouncer(0.25, DebounceType.kRising);

  @Getter
  @AutoLogOutput(key = "Elevator/Profile/AtGoal")
  private boolean atGoal = false;

  @Setter private boolean stowed = false;

  static {
    switch (Constants.getRobot()) {
      case COMPBOT, DEVBOT -> {
        kP.initDefault(0);
        kD.initDefault(0);
        kS.initDefault(0);
        kG.initDefault(0);
        kV.initDefault(0);
        kA.initDefault(0);
      }
      case SIMBOT -> {
        kP.initDefault(5000);
        kD.initDefault(2000);
        kS.initDefault(0);
        kG.initDefault(0);
        kV.initDefault(0);
        kA.initDefault(0);
      }
    }
  }

  public Elevator(ElevatorIO elevatorIO) {

    this.elevatorIO = elevatorIO;

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                maxVelocityMeterPerSec.get(), maxAccelerationMeterPerSec2.get()));
  }

  public void periodic() {

    elevatorIO.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    leaderMotorDisconnectedAlert.set(!inputs.data.motorConnected() && !Robot.isJITing());
    followerMotorDisconnectedAlert.set(!inputs.data.followerConnected() && !Robot.isJITing());

    // Update tunable numbers
    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      elevatorIO.setPID(kP.get(), 0.0, kD.get());
    }

    if (maxVelocityMeterPerSec.hasChanged(hashCode())) {
      profile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  maxVelocityMeterPerSec.get(), maxAccelerationMeterPerSec2.get()));
    }

    // Run Profile
    final boolean shouldRunProfile =
        !stopProfile
            && !coastOverride.getAsBoolean()
            && !disabledOverride.getAsBoolean()
            && homed
            && !isEStopped
            && DriverStation.isEnabled();
    Logger.recordOutput("Elevator/RunningProfile", shouldRunProfile);

    // Check if out of tolerance
    boolean outOfTolerance =
        Math.abs(inputs.data.positionRad() - setpoint.position) > tolerance.get();

    // Check if should E-Stop
    shouldEStop = toleranceDebouncer.calculate(outOfTolerance && shouldRunProfile);

    if (shouldRunProfile) {

      // Clamp Goal
      var goalState =
          new State(
              MathUtil.clamp(goal.get().position, 0.0, ElevatorConstants.maxTravel),
              goal.get().velocity);

      double previousVelocity = setpoint.velocity;

      setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);

      if (setpoint.position < 0.0 || setpoint.position > ElevatorConstants.maxTravel) {
        setpoint =
            new State(MathUtil.clamp(setpoint.position, 0.0, ElevatorConstants.maxTravel), 0.0);
      }

      double accel = (setpoint.velocity - previousVelocity) / Constants.loopPeriodSecs;

      elevatorIO.runPosition(
          (setpoint.position / sprocketRadius) + homePositionRad,
          kS.get() * Math.signum(setpoint.velocity) + kG.get() + kA.get() * accel);

      atGoal =
          EqualsUtil.epsilonEquals(setpoint.position, goalState.position)
              && EqualsUtil.epsilonEquals(setpoint.velocity, goalState.velocity);

      // Log State
      Logger.recordOutput("Elevator/Profile/SetpointPositionMeter", setpoint.position);
      Logger.recordOutput("Elevator/Profile/SetpointVelocityMeterPerSec", setpoint.velocity);
      Logger.recordOutput("Elevator/Profile/GoalPositionMeters", goalState.position);
      Logger.recordOutput("Elevator/Profile/GoalVelocityMeterPerSec", goalState.velocity);

    } else {

      // Reset setpoint
      setpoint = new State(getPositionMeters(), 0.0);

      // Clear logs
      Logger.recordOutput("Elevator/Profile/SetpointPositionMeter", 0.0);
      Logger.recordOutput("Elevator/Profile/SetpointVelocityMeterPerSec", 0.0);
      Logger.recordOutput("Elevator/Profile/GoalPositionMeters", 0.0);
      Logger.recordOutput("Elevator/Profile/GoalVelocityMeterPerSec", 0.0);
    }

    if (isEStopped) {
      elevatorIO.stop();
    }

    // Log state
    Logger.recordOutput("Elevator/CoastOverride", coastOverride.getAsBoolean());
    Logger.recordOutput("Elevator/DisableOverride", disabledOverride.getAsBoolean());
    Logger.recordOutput(
        "Elevator/MeasuredVelocityMeterPerSec", inputs.data.velocityRadPerSec() * sprocketRadius);

    // Record cycle time
    LoggedTracer.record("Elevator");
  }

  public void setGoal(DoubleSupplier goal) {
    setGoal(() -> new State(goal.getAsDouble(), 0.0));
  }

  public void setGoal(Supplier<State> goal) {
    atGoal = false;
    this.goal = goal;
  }

  public double getHomePos() {
    return homePositionRad * 1;
  }

  public double getMaxTravel() {
    return (ElevatorConstants.maxTravel / 1) + homePositionRad;
  }

  public void setOverrides(BooleanSupplier coastOverride, BooleanSupplier disabledOverride) {
    this.coastOverride = coastOverride;
    this.disabledOverride = disabledOverride;
  }

  private void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) {
      return;
    }
    brakeModeEnabled = enabled;
    elevatorIO.setBrakeMode(brakeModeEnabled);
  }

  public void setHome() {
    homePositionRad = inputs.data.positionRad();
    homed = true;
  }

  public Command homingSequence() {
    return Commands.startRun(
            () -> {
              stopProfile = true;
              homed = false;
              homingDebouncer = new Debouncer(homingTimeSecs.get());
              homingDebouncer.calculate(false);
            },
            () -> {
              if (disabledOverride.getAsBoolean() || coastOverride.getAsBoolean()) return;
              elevatorIO.runVolts(homingVolts.get());
              homed =
                  homingDebouncer.calculate(
                      Math.abs(inputs.data.velocityRadPerSec()) <= homingVelocityThresh.get());
            })
        .until(() -> homed)
        .andThen(this::setHome)
        .finallyDo(
            () -> {
              stopProfile = false;
            });
  }

  public Command upStaticCharacterization() {
    final StaticCharacterizationState state = new StaticCharacterizationState();
    Timer timer = new Timer();
    return Commands.startRun(
            () -> {
              stopProfile = true;
              timer.restart();
            },
            () -> {
              stopProfile = true;
              state.characterizationOutput = characterizationRampRate.get() * timer.get();
              elevatorIO.runOpenLoop(state.characterizationOutput);
              Logger.recordOutput(
                  "Elevator/CharacterizationOutputUp", state.characterizationOutput);
            })
        .until(() -> inputs.data.velocityRadPerSec() >= characterizationUpVelocityThresh.get())
        .andThen(elevatorIO::stop)
        .andThen(Commands.idle())
        .finallyDo(
            () -> {
              stopProfile = false;
              timer.stop();
              Logger.recordOutput(
                  "Elevator/CharacterizationOutputUp", state.characterizationOutput);
            });
  }

  public Command downStaticCharacterization() {
    final StaticCharacterizationState state = new StaticCharacterizationState();
    Timer timer = new Timer();
    return Commands.startRun(
            () -> {
              stopProfile = true;
              timer.restart();
            },
            () -> {
              state.characterizationOutput =
                  characterizationDownStartAmps.get()
                      - characterizationRampRate.get() * timer.get();
              elevatorIO.runOpenLoop(state.characterizationOutput);
              Logger.recordOutput(
                  "Elevator/CharacterizationOutputDown", state.characterizationOutput);
            })
        .until(() -> inputs.data.velocityRadPerSec() <= characterizationDownVelocityThresh.get())
        .andThen(elevatorIO::stop)
        .andThen(Commands.idle())
        .finallyDo(
            () -> {
              stopProfile = false;
              timer.stop();
              Logger.recordOutput(
                  "Elevator/CharacterizationOutputDown", state.characterizationOutput);
            });
  }

  @AutoLogOutput(key = "Elevator/MeasuredHeightMeters")
  public double getPositionMeters() {
    return (inputs.data.positionRad() - homePositionRad) * sprocketRadius;
  }

  public double getGoalMeters() {
    return goal.get().position;
  }

  public void runVolts(double volt) {
    elevatorIO.runVolts(volt);
  }

  private static class StaticCharacterizationState {
    public double characterizationOutput = 0.0;
  }
}
