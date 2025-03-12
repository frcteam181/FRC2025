package frc.robot.subsystems.superstructure.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  // Holds sensor inputs
  @AutoLog
  class ElevatorIOInputs {
    public ElevatorIOData data =
        new ElevatorIOData(false, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  record ElevatorIOData(
      boolean motorConnected,
      boolean followerConnected,
      double positionRad,
      double velocityRadPerSec,
      double appliedVolts,
      double torqueCurrentAmps,
      double supplyCurrentAmps,
      double tempCelsius,
      double followerAppliedVolts,
      double followerTorqueCurrentAmps,
      double followerSupplyCurrentAmps,
      double followerTempCelsius) {}

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void runOpenLoop(double output) {}

  default void runVolts(double volts) {}

  default void stop() {}

  default void runPosition(double positionRad, double feedforward) {}

  default void runVoltPosition(double positionRad, double feedforward) {}

  default void setPID(double kP, double kI, double kD) {}

  default void setBrakeMode(boolean enabled) {}
}
