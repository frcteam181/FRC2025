package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollerSystemIO {

  @AutoLog
  class RollerSystemIOInputs {
    public RollerSystemIOData data =
        new RollerSystemIOData(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, false);
  }

  record RollerSystemIOData(
      double positionRads,
      double velocityRadsPerSec,
      double appliedVoltage,
      double supplyCurrentAmps,
      double torqueCurrentAmps,
      double tempCelsius,
      boolean tempFault,
      boolean connected) {}

  default void updateInputs(RollerSystemIOInputs inputs) {}

  default void runVolts(double volts) {}

  default void stop() {}

  default void setCurrentLimit(double currentLimit) {}

  default void setPID(double kP, double kI, double kD) {}

  default void setBrakeMode(boolean enabled) {}
}
