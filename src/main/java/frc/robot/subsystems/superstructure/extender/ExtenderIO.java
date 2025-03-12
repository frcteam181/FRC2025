package frc.robot.subsystems.superstructure.extender;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ExtenderIO {
  @AutoLog
  class ExtenderIOInputs {
    public ExtenderIOData data =
        new ExtenderIOData(
            false, Rotation2d.kZero, 0.0, 0.0, 0.0, 0.0, 0.0, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  public record ExtenderIOData(
      boolean pivotConnected,
      Rotation2d pivotPosition,
      double pivotVelocityRadPerSec,
      double pivotAppliedVolts,
      double pivotSupplyCurrentAmps,
      double pivotTorqueCurrentAmps,
      double pivotTempCelcius,
      boolean rollerConnected,
      double rollerPosition,
      double rollerVelocity,
      double rollerAppliedVolts,
      double rollerSupplyCurrentAmps,
      double rollerTorqueCurrentAmps,
      double rollerTempCelcius) {}

  // Extender Methods

  default void updateInputs(ExtenderIOInputs inputs) {}

  // Pivot Methods

  default void runPivotOpenLoop(double output) {}

  default void runPivotVolts(double volts) {}

  default void stopPivot() {}

  default void runPivotPosition(Rotation2d position, double feedforward) {}

  default void setPivotPID(double kP, double kI, double kD) {}

  default void setPivotBrakeMode(boolean enabled) {}

  // Roller Methods

  default void runRollerVolts(double volts) {}

  default void stopRoller() {}

  default void setRollerBrakeMode(boolean enabled) {}
}
