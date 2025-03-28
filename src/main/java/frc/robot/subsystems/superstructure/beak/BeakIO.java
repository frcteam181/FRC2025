package frc.robot.subsystems.superstructure.beak;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface BeakIO {

  // Holds sensor inputs
  @AutoLog
  class PivotIOInputs {
    public PivotIOData data = new PivotIOData(false, Rotation2d.kZero, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  record PivotIOData(
      boolean motorConnected,
      Rotation2d motorPosition,
      double motorVelocityRadPerSec,
      double motorAppliedVolts,
      double motorSupplyCurrentAmps,
      double motorTorqueCurrentAmps,
      double motorTempCelcius) {}

  default void updateInputs(PivotIOInputs inputs) {}

  default void runOpenLoop(double output) {}

  default void runVolts(double volts) {}

  default void stop() {}

  /** Run elevator output shaft to positionRad with additional feedforward output */
  default void runPosition(Rotation2d positionRad, double feedforward) {}

  default void setPID(double kP, double kI, double kD) {}

  default void setBrakeMode(boolean enabled) {}
}
