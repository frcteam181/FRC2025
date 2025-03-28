package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  class ClimberIOInputs {
    public ClimberIOData data = new ClimberIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  record ClimberIOData(
      boolean motorConnected,
      double motorPositionRads,
      double motorVelocityRadsPerSec,
      double motorAppliedVoltage,
      double motorSupplyCurrentAmps,
      double motorTorqueCurrentAmps,
      double motorTempCelsius) {}

  default void updateInputs(ClimberIOInputs inputs) {}

  default void runTorqueCurrent(double current) {}

  default void runVolts(double volts) {}

  default void stop() {}

  default void setBrakeMode(boolean enabled) {}
}
