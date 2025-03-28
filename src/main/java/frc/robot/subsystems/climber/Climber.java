package frc.robot.subsystems.climber;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private static final LoggedTunableNumber deployCurrent =
      new LoggedTunableNumber("Climber/DeployCurrent", 0.0);
  private static final LoggedTunableNumber deployAngle =
      new LoggedTunableNumber("Climber/DeployAngle", 0.0);
  private static final LoggedTunableNumber undeployAngle =
      new LoggedTunableNumber("Climber/UndeployAngle", 0.0);
  private static final LoggedTunableNumber climbCurrent =
      new LoggedTunableNumber("Climber/ClimbCurrent", 0.0);
  private static final LoggedTunableNumber climbCurrentRampRate =
      new LoggedTunableNumber("Climber/ClimbCurrentRampRate", 0.0);
  private static final LoggedTunableNumber climbStopAngle =
      new LoggedTunableNumber("Climber/ClimbStopAngle", 0.0);

  private final ClimberIO climberIO;
  private final ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

  @Setter private BooleanSupplier coastOverride = () -> false;
  @AutoLogOutput private double climbStopOffsetDegrees = 0.0;

  @AutoLogOutput(key = "CLimber/BrakeModeEnabled")
  private boolean brakeModeEnabled = true;

  @AutoLogOutput private ClimbState climbState = ClimbState.START;

  private final Timer climbTimer = new Timer();

  private final Alert climberDisconnected =
      new Alert("Climber motor Disconnected!", Alert.AlertType.kWarning);

  public Climber(ClimberIO climberIO) {
    this.climberIO = climberIO;
    climberIO.setBrakeMode(true);
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(climberInputs);
    Logger.processInputs("Climber/Climber", climberInputs);

    climberDisconnected.set(!climberInputs.data.motorConnected() && !Robot.isJITing());

    if (DriverStation.isDisabled()) {
      climberIO.runTorqueCurrent(0.0);
      climbState = ClimbState.START;
    }

    boolean coast = coastOverride.getAsBoolean() && DriverStation.isDisabled();
    setBrakeMode(!coast);
    Logger.recordOutput("Climber/CoastOverride", !coast);

    switch (climbState) {
      case START -> {}
      case READY -> {
        double position = climberInputs.data.motorPositionRads();

        if (position <= Units.degreesToRadians(deployAngle.get())) {
          climberIO.runTorqueCurrent(deployCurrent.get());
        } else if (position <= Units.degreesToRadians(undeployAngle.get())) {
          climberIO.stop();
        } else {
          climberIO.runTorqueCurrent(-deployCurrent.get());
        }
      }
      case PULL -> {
        boolean stopped =
            climberInputs.data.motorPositionRads()
                >= Units.degreesToRadians(climbStopAngle.get() + climbStopOffsetDegrees);
        if (stopped) {
          climbTimer.restart();
        }
        climberIO.runTorqueCurrent(
            stopped
                ? 0.0
                : Math.min(climbCurrentRampRate.get() * climbTimer.get(), climbCurrent.get()));
      }
    }

    LoggedTracer.record("Climber");
  }

  public Command readyClimb() {
    return runOnce(
        () -> {
          if (climbState == ClimbState.READY) {
            climbTimer.restart();
            climbState = ClimbState.PULL;
          } else {
            climbState = ClimbState.READY;
          }
        });
  }

  public void adjustClimbOffset(double offset) {
    climbStopOffsetDegrees += offset;
  }

  private void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    climberIO.setBrakeMode(enabled);
  }

  public void runVolts(double volts) {
    climberIO.runVolts(volts);
  }

  public enum ClimbState {
    START,
    READY,
    PULL
  }
}
