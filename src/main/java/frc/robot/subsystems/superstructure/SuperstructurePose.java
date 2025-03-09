package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public enum SuperstructurePose {
  START(0.0, Rotation2d.kZero, Rotation2d.kZero),
  AUTO_START(0.0, Rotation2d.kZero, Rotation2d.kZero),
  STOW(0.0, Rotation2d.fromDegrees(Units.degreesToRadians((20.0))), Rotation2d.kZero),
  L1_CORAL(0.0, Rotation2d.kZero, Rotation2d.kZero),
  L2_CORAL(0.0, Rotation2d.kZero, Rotation2d.kZero),
  L3_CORAL(0.0, Rotation2d.kZero, Rotation2d.kZero),
  L4_CORAL(0.0, Rotation2d.kZero, Rotation2d.kZero),
  L1_CORAL_EJECT(0.0, Rotation2d.kZero, Rotation2d.kZero),
  L2_CORAL_EJECT(0.0, Rotation2d.kZero, Rotation2d.kZero),
  L3_CORAL_EJECT(0.0, Rotation2d.kZero, Rotation2d.kZero),
  L4_CORAL_EJECT(0.0, Rotation2d.kZero, Rotation2d.kZero),
  ALGAE_FLOOR_INTAKE(0.0, Rotation2d.kZero, Rotation2d.kZero),
  ALGAE_L2_INTAKE(0.0, Rotation2d.kZero, Rotation2d.kZero),
  ALGAE_L3_INTAKE(0.0, Rotation2d.kZero, Rotation2d.kZero),
  NET(0.0, Rotation2d.kZero, Rotation2d.kZero);

  private final double HEIGHT;
  private final Rotation2d EXT_ANGLE;
  private final Rotation2d BEAK_ANGLE;

  SuperstructurePose(double HEIGHT, Rotation2d EXT_ANGLE, Rotation2d BEAK_ANGLE) {
    this.HEIGHT = HEIGHT;
    this.EXT_ANGLE = EXT_ANGLE;
    this.BEAK_ANGLE = BEAK_ANGLE;
  }

  public double getHEIGHT() {
    return HEIGHT;
  }

  public Rotation2d getEXT_ANGLE() {
    return EXT_ANGLE;
  }

  public Rotation2d getBeakAngle() {
    return BEAK_ANGLE;
  }
}
