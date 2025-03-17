package frc.robot.subsystems.superstructure;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import lombok.Getter;

public enum SuperstructurePose {
  START(
      0.0,
      Rotation2d.fromDegrees(67.0),
      Rotation2d.fromDegrees(67.0),
      Rotation2d.kZero,
      Rotation2d.kZero),
  AUTO_START(
      0.0,
      Rotation2d.fromDegrees(67.0),
      Rotation2d.fromDegrees(67.0),
      Rotation2d.kZero,
      Rotation2d.kZero),
  STOW(
      0.0,
      Rotation2d.fromDegrees(67.0),
      Rotation2d.fromDegrees(0.0),
      Rotation2d.kZero,
      Rotation2d.kZero),
  L1_CORAL(
      Units.inchesToMeters(12.0),
      Rotation2d.fromDegrees(90.0),
      Rotation2d.kZero,
      Rotation2d.kZero,
      Rotation2d.kZero),
  L2_CORAL(
      Units.inchesToMeters(24.0),
      Rotation2d.fromDegrees(90.0),
      Rotation2d.kZero,
      Rotation2d.kZero,
      Rotation2d.kZero),
  L3_CORAL(
      Units.inchesToMeters(36.0),
      Rotation2d.fromDegrees(90.0),
      Rotation2d.kZero,
      Rotation2d.kZero,
      Rotation2d.kZero),
  L4_CORAL(
      Units.inchesToMeters(48.0),
      Rotation2d.fromDegrees(90.0),
      Rotation2d.kZero,
      Rotation2d.kZero,
      Rotation2d.kZero),
  L1_CORAL_EJECT(
      0.0, Rotation2d.fromDegrees(90.0), Rotation2d.kZero, Rotation2d.kZero, Rotation2d.kZero),
  L2_CORAL_EJECT(
      0.0, Rotation2d.fromDegrees(90.0), Rotation2d.kZero, Rotation2d.kZero, Rotation2d.kZero),
  L3_CORAL_EJECT(
      0.0, Rotation2d.fromDegrees(90.0), Rotation2d.kZero, Rotation2d.kZero, Rotation2d.kZero),
  L4_CORAL_EJECT(
      0.0, Rotation2d.fromDegrees(90.0), Rotation2d.kZero, Rotation2d.kZero, Rotation2d.kZero),
  ALGAE_FLOOR_INTAKE(
      0.0,
      Rotation2d.fromDegrees(90.0),
      Rotation2d.fromDegrees(-10),
      Rotation2d.kZero,
      Rotation2d.kZero),
  ALGAE_L2_INTAKE(
      Units.inchesToMeters(15.0),
      Rotation2d.fromDegrees(67.0),
      Rotation2d.fromDegrees(40.0),
      Rotation2d.kZero,
      Rotation2d.kZero),
  ALGAE_L3_INTAKE(
      Units.inchesToMeters(32.0),
      Rotation2d.fromDegrees(67.0),
      Rotation2d.fromDegrees(40.0),
      Rotation2d.kZero,
      Rotation2d.kZero),
  NET(
      ElevatorConstants.maxTravel,
      Rotation2d.fromDegrees(67.0),
      Rotation2d.fromDegrees(90.0),
      Rotation2d.kZero,
      Rotation2d.kZero);

  @Getter private final double HEIGHT;
  @Getter private final Rotation2d EXTENDER_STOW_ANGLE;
  @Getter private final Rotation2d EXTENDER_OUT_ANGLE;
  @Getter private final Rotation2d BEAK_STOW_ANGLE;
  @Getter private final Rotation2d BEAK_OUT_ANGLE;

  SuperstructurePose(
      double HEIGHT,
      Rotation2d EXTENDER_STOW_ANGLE,
      Rotation2d EXTENDER_OUT_ANGLE,
      Rotation2d BEAK_STOW_ANGLE,
      Rotation2d BEAK_OUT_ANGLE) {
    this.HEIGHT = HEIGHT;
    this.EXTENDER_STOW_ANGLE = EXTENDER_STOW_ANGLE;
    this.EXTENDER_OUT_ANGLE = EXTENDER_OUT_ANGLE;
    this.BEAK_STOW_ANGLE = BEAK_STOW_ANGLE;
    this.BEAK_OUT_ANGLE = BEAK_OUT_ANGLE;
  }

  // public double getElevatorPoseHeight() {
  //   return HEIGHT;
  // }

  // public Rotation2d getExtenderStowAngle() {
  //   return EXTENDER_STOW_ANGLE;
  // }

  // public Rotation2d getExtenderOutAngle() {
  //   return EXTENDER_OUT_ANGLE;
  // }

  // public Rotation2d getBeakStowAngle() {
  //   return BEAK_STOW_ANGLE;
  // }

  // public Rotation2d getBeakOutAngle() {
  //   return BEAK_OUT_ANGLE;
  // }
}
