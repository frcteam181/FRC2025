package frc.robot.subsystems.superstructure;

import static frc.robot.subsystems.superstructure.SuperstructurePose.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.SuperstructureConstants.ElevatorConstants;
import frc.robot.subsystems.superstructure.beak.Beak;
import frc.robot.subsystems.superstructure.beak.Beak.RollerGoal;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.extender.Extender;
import frc.robot.subsystems.superstructure.extender.Extender.GripperGoal;
import lombok.Setter;

public class Superstructure extends SubsystemBase {

  private final Elevator elevator;
  private final Extender extender;
  private final Beak beak;

  private enum SuperstructureState {
    START,
    AUTO_START,
    STOW,
    L1_CORAL,
    L2_CORAL,
    L3_CORAL,
    L4_CORAL,
    L1_CORAL_EJECT,
    L2_CORAL_EJECT,
    L3_CORAL_EJECT,
    L4_CORAL_EJECT,
    ALGAE_FLOOR_INTAKE,
    ALGAE_L2_INTAKE,
    ALGAE_L3_INTAKE,
    NET
  }

  private SuperstructureState previous = null;
  private SuperstructureState state = SuperstructureState.START;
  private SuperstructureState next = null;
  private SuperstructureState goal = SuperstructureState.START;

  @Setter private boolean shouldExtend = false;

  public Superstructure(Elevator elevator, Extender extender, Beak beak) {
    this.elevator = elevator;
    this.extender = extender;
    this.beak = beak;
  }

  @Override
  public void periodic() {
    elevator.periodic();
    extender.periodic();
    beak.periodic();
    // calculateState();
  }

  // Elevator Methods
  public void runElevatorOpenLoopVolt(double volt) {
    elevator.runVolts(volt);
  }

  public void sendElevatorHome() {
    elevator.setGoal(() -> new State(elevator.getHomePosMeter(), 0.0));
  }

  public void sendElevatorToPercent() {
    elevator.setGoal(() -> new State((ElevatorConstants.maxTravel * 0.95), 0.0));
  }

  public Command homeElevator() {
    return elevator.homingSequence();
  }

  // Extender Methods
  public void sendExtenderHome() {
    extender.setPivotGoal(() -> Rotation2d.fromDegrees(90.0));
  }

  public void sendExtenderToHor() {
    extender.setPivotGoal(() -> Rotation2d.fromDegrees(0.0));
  }

  public Command homeExtender() {
    return extender.homingSequence();
  }

  public void startManipulatingAlgae() {
    extender.setShouldManipulateAlgae(true);
  }

  public void stopManipulatingAlgae() {
    extender.setShouldManipulateAlgae(false);
  }

  public void ejectAlgae() {
    extender.setGripperGoal(GripperGoal.EJECT);
  }
  // public void ejectAlgae() {
  //   extender.setGripperGoal(state == SuperstructureState.NET ? GripperGoal.NET_EJECT :
  // GripperGoal.EJECT);
  // }

  // Beak Methods
  public void startManipulatingCoral() {
    beak.setShouldManipulateCoral(true);
  }

  public void stopManipulatingCoral() {
    beak.setShouldManipulateCoral(false);
  }

  public void ejectCoral() {
    beak.setRollerGoal(RollerGoal.EJECT);
  }
  // public void ejectCoral() {
  //   beak.setRollerGoal(state == SuperstructureState.L1_CORAL ? RollerGoal.L1_EJECT :
  // RollerGoal.EJECT);
  // }

  // Superstructure Methods
  public void startManipulatingGamePieces() {
    startManipulatingAlgae();
    startManipulatingCoral();
  }

  public void stopManipulatingGamePieces() {
    stopManipulatingAlgae();
    stopManipulatingCoral();
  }

  public void calculateState() {
    switch (goal) {
        /* ----------------------------------------START---------------------------------------- */
      case START:
        elevator.homingSequence();

        if (elevator.isHomed()) {
          previous = state;
          next = SuperstructureState.STOW;
        }
        break;
        /* -------------------------------------AUTO_START---------------------------------------- */
      case AUTO_START:
        break;
        /* -------------------------------------STOW---------------------------------------- */
      case STOW:
        elevator.setGoal(() -> STOW.getHEIGHT());
        beak.setGoal(
            () -> STOW.getBeakAngle()); // Change this so beak has the stowed angle and the extended
        // angle
        if (shouldExtend) {
          extender.setPivotGoal(() -> STOW.getEXT_ANGLE());
        }
        break;
        /* -------------------------------------L1 CORAL---------------------------------------- */
      case L1_CORAL:
        elevator.setGoal(() -> L1_CORAL.getHEIGHT());
        beak.setGoal(() -> L1_CORAL.getBeakAngle());
        if (shouldExtend) {
          extender.setPivotGoal(() -> L1_CORAL.getEXT_ANGLE());
          break;
        }
        extender.setPivotGoal(() -> STOW.getEXT_ANGLE());
        break;
        /* -------------------------------------L2 CORAL---------------------------------------- */
      case L2_CORAL:
        elevator.setGoal(() -> L2_CORAL.getHEIGHT());
        beak.setGoal(() -> L2_CORAL.getBeakAngle());
        if (shouldExtend) {
          extender.setPivotGoal(() -> L2_CORAL.getEXT_ANGLE());
          break;
        }
        extender.setPivotGoal(() -> STOW.getEXT_ANGLE());
        break;
        /* -------------------------------------L3 CORAL---------------------------------------- */
      case L3_CORAL:
        elevator.setGoal(() -> L3_CORAL.getHEIGHT());
        beak.setGoal(() -> L3_CORAL.getBeakAngle());
        if (shouldExtend) {
          extender.setPivotGoal(() -> L3_CORAL.getEXT_ANGLE());
          break;
        }
        extender.setPivotGoal(() -> STOW.getEXT_ANGLE());
        break;
        /* -------------------------------------L4 CORAL---------------------------------------- */
      case L4_CORAL:
        elevator.setGoal(() -> L4_CORAL.getHEIGHT());
        beak.setGoal(() -> L4_CORAL.getBeakAngle());
        if (shouldExtend) {
          extender.setPivotGoal(() -> L4_CORAL.getEXT_ANGLE());
          break;
        }
        extender.setPivotGoal(() -> STOW.getEXT_ANGLE());
        break;
        /* -------------------------------------L1 CORAL EJECT---------------------------------------- */
      case L1_CORAL_EJECT:
        elevator.setGoal(() -> L1_CORAL.getHEIGHT());
        beak.setGoal(() -> L1_CORAL.getBeakAngle());
        if (shouldExtend) {
          extender.setPivotGoal(() -> L1_CORAL.getEXT_ANGLE());
          break;
        }
        extender.setPivotGoal(() -> STOW.getEXT_ANGLE());
        break;
        /* -------------------------------------L2 CORAL EJECT---------------------------------------- */
      case L2_CORAL_EJECT:
        elevator.setGoal(() -> L2_CORAL.getHEIGHT());
        beak.setGoal(() -> L2_CORAL.getBeakAngle());
        if (shouldExtend) {
          extender.setPivotGoal(() -> L2_CORAL.getEXT_ANGLE());
          break;
        }
        extender.setPivotGoal(() -> STOW.getEXT_ANGLE());
        break;
        /* -------------------------------------L3 CORAL EJECT---------------------------------------- */
      case L3_CORAL_EJECT:
        elevator.setGoal(() -> L3_CORAL.getHEIGHT());
        beak.setGoal(() -> L3_CORAL.getBeakAngle());
        if (shouldExtend) {
          extender.setPivotGoal(() -> L3_CORAL.getEXT_ANGLE());
          break;
        }
        extender.setPivotGoal(() -> STOW.getEXT_ANGLE());
        break;
        /* -------------------------------------L4 CORAL EJECT---------------------------------------- */
      case L4_CORAL_EJECT:
        elevator.setGoal(() -> L4_CORAL.getHEIGHT());
        beak.setGoal(() -> L4_CORAL.getBeakAngle());
        if (shouldExtend) {
          extender.setPivotGoal(() -> L4_CORAL.getEXT_ANGLE());
          break;
        }
        extender.setPivotGoal(() -> STOW.getEXT_ANGLE());
        break;
        /* -------------------------------------ALGAE FLOOR INTAKE---------------------------------------- */
      case ALGAE_L2_INTAKE:
        elevator.setGoal(() -> ALGAE_L2_INTAKE.getHEIGHT());
        beak.setGoal(() -> ALGAE_L2_INTAKE.getBeakAngle());
        if (shouldExtend) {
          extender.setPivotGoal(() -> ALGAE_L2_INTAKE.getEXT_ANGLE());
          break;
        }
        extender.setPivotGoal(() -> STOW.getEXT_ANGLE());
        break;
        /* -------------------------------------ALGAE L2 INTAKE---------------------------------------- */
      case ALGAE_L3_INTAKE:
        elevator.setGoal(() -> ALGAE_L3_INTAKE.getHEIGHT());
        beak.setGoal(() -> ALGAE_L3_INTAKE.getBeakAngle());
        if (shouldExtend) {
          extender.setPivotGoal(() -> ALGAE_L3_INTAKE.getEXT_ANGLE());
          break;
        }
        extender.setPivotGoal(() -> STOW.getEXT_ANGLE());
        break;
        /* -------------------------------------NET---------------------------------------- */
      case NET:
        elevator.setGoal(() -> NET.getHEIGHT());
        beak.setGoal(() -> NET.getBeakAngle());
        if (shouldExtend) {
          extender.setPivotGoal(() -> NET.getEXT_ANGLE());
          break;
        }
        extender.setPivotGoal(() -> STOW.getEXT_ANGLE());
        break;

      default:
        break;
    }
  }
}
