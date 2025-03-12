package frc.robot.subsystems.superstructure.extender;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ExtenderIO {
  @AutoLog
  class ExtenderIOInputs {
    public ExtenderIOData data =
        new ExtenderIOData(false, Rotation2d.kZero, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  public record ExtenderIOData(
      boolean motorConnected,
      Rotation2d motorPosition,
      double motorVelocityRadPerSec,
      double motorAppliedVolts,
      double motorSupplyCurrentAmps,
      double motorTorqueCurrentAmps,
      double motorTempCelcius) {}

  // Extender Methods

  default void updateInputs(ExtenderIOInputs inputs) {}

  // Motor Methods

  default void runOpenLoop(double output) {}

  default void runVolts(double volts) {}

  default void stop() {}

  default void runPosition(Rotation2d position, double feedforward) {}

  default void setPID(double kP, double kI, double kD) {}

  default void setBrakeMode(boolean enabled) {}
}
