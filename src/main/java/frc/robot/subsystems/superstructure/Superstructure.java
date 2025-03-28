package frc.robot.subsystems.superstructure;

import static frc.robot.subsystems.superstructure.SuperstructurePose.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.beak.Beak;
import frc.robot.subsystems.superstructure.beak.Beak.RollerGoal;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.extender.Extender;
import frc.robot.subsystems.superstructure.extender.Extender.GripperGoal;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Superstructure extends SubsystemBase {

  private final Elevator elevator;
  private final Extender extender;
  private final Beak beak;

  // private enum SuperstructureState {
  //   START,
  //   AUTO_START,
  //   STOW,
  //   L1_CORAL,
  //   L2_CORAL,
  //   L3_CORAL,
  //   L4_CORAL,
  //   L1_CORAL_EJECT,
  //   L2_CORAL_EJECT,
  //   L3_CORAL_EJECT,
  //   L4_CORAL_EJECT,
  //   ALGAE_FLOOR_INTAKE,
  //   ALGAE_L2_INTAKE,
  //   ALGAE_L3_INTAKE,
  //   NET
  // }

  // private SuperstructureState previousState = null;
  // private SuperstructureState currentState = null;
  // @AutoLogOutput @Setter @Getter private SuperstructureState goal;

  private SuperstructurePose previousPose = null;
  private SuperstructurePose currentPose = null;
  @AutoLogOutput @Setter @Getter private SuperstructurePose pose;

  @AutoLogOutput @Setter private boolean shouldExtend = false;
  @AutoLogOutput private boolean isExtending = false;
  @AutoLogOutput private boolean beakOut = false;

  public Superstructure(Elevator elevator, Extender extender, Beak beak) {
    this.elevator = elevator;
    this.extender = extender;
    this.beak = beak;

    // setGoal(SuperstructureState.START);
    setPose(SuperstructurePose.START);
  }

  @Override
  public void periodic() {
    elevator.periodic();
    extender.periodic();
    beak.periodic();

    if (!elevator.isHomed()) {
      // elevator.homingSequence().schedule();
    } else {
      if (elevator.isHomed() && getPose() == SuperstructurePose.START) {
        setPose(SuperstructurePose.STOW);
      }
      calculateElevator();
      calculateExtender();
      // calculateBeak();
    }
    // calculateState();
  }

  // Elevator Methods
  public void runElevatorOpenLoopVolt(double volt) {
    elevator.runVolts(volt);
  }

  public Command homeElevator() {
    return elevator.homingSequence();
  }

  public void switchElevatorBreakOverride() {
    if (elevator.isBrakeEnabled()) {
      elevator.setOverrides(() -> true, () -> false);
    } else {
      elevator.setOverrides(() -> false, () -> false);
    }
  }

  // Extender Methods
  public void sendExtenderHome() {
    extender.setPivotGoal(() -> Rotation2d.fromDegrees(90.0));
  }

  public void sendExtenderToHor() {
    extender.setPivotGoal(() -> Rotation2d.fromDegrees(0.0));
  }

  public void runExtenderOpenLoopVolt(double volt) {
    extender.runPivotVolts(volt);
  }

  // public Command homeExtender() {
  //   return extender.homingSequence();
  // }

  public void startManipulatingAlgae() {
    extender.setShouldManipulateAlgae(true);
  }

  public void stopManipulatingAlgae() {
    extender.setShouldManipulateAlgae(false);
  }

  public void ejectAlgae() {
    extender.startEjectTimer(); // -----------------------------------
    extender.setGripperGoal(GripperGoal.EJECT); // Swapped the order of these two lines
  }

  public void switchExtenderPivotBreakOverride() {
    if (extender.isPivotBrakeEnabled()) {
      extender.setOverrides(() -> true, () -> false, () -> false);
    } else {
      extender.setOverrides(() -> false, () -> false, () -> false);
    }
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
    beak.startEjectTimer(); // -----------------------------------
    beak.setRollerGoal(RollerGoal.EJECT); // Swapped the order of these two lines
  }

  public void sendBeakHome() {
    beak.setGoal(() -> Rotation2d.fromDegrees(0.0));
  }

  public void sendBeakToMax() {
    beak.setGoal(() -> Rotation2d.fromDegrees(-180.0));
  }

  public void runBeakPivotOpenLoop(double output) {
    beak.runPivotOpenLoop(output);
  }

  public void switchBeakPivotBreakOverride() {
    if (beak.isPivotBrakeEnabled()) {
      beak.setOverrides(() -> true, () -> false, () -> false);
    } else {
      beak.setOverrides(() -> false, () -> false, () -> false);
    }
  }
  // public void ejectCoral() {
  //   beak.setRollerGoal(state == SuperstructureState.L1_CORAL ? RollerGoal.L1_EJECT :
  // RollerGoal.EJECT);
  // }

  // Superstructure Methods
  public void startManipulatingGamePieces() {
    startManipulatingAlgae();
    // startManipulatingCoral();
    setShouldExtend(true);
  }

  public void stopManipulatingGamePieces() {
    stopManipulatingAlgae();
    stopManipulatingCoral();
    setShouldExtend(false);
  }

  public void ELEVATOR_TO_STOW() {
    setPose(SuperstructurePose.STOW);
  }

  public void ELEVATOR_TO_ALGAE_L2_INTAKE() {
    setPose(SuperstructurePose.ALGAE_L2_INTAKE);
  }

  public void ELEVATOR_TO_ALGAE_L3_INTAKE() {
    setPose(SuperstructurePose.ALGAE_L3_INTAKE);
  }

  public void ELEVATOR_TO_NET() {
    setPose(SuperstructurePose.NET);
  }

  public void calculateElevator() {
    if (previousPose != pose) {
      elevator.setGoal(() -> getPose().getHEIGHT());
      previousPose = pose;
    }
  }

  public void calculateExtender() {
    if (shouldExtend && !isExtending) {
      extender.setPivotGoal(() -> getPose().getEXTENDER_OUT_ANGLE());
      isExtending = true;
    } else if (isExtending && !shouldExtend) {
      extender.setPivotGoal(() -> getPose().getEXTENDER_STOW_ANGLE());
      isExtending = false;
    }
  }

  public void calculateBeak() {
    if (isExtending && !beakOut) {
      beak.setGoal(() -> getPose().getBEAK_OUT_ANGLE());
      beakOut = true;
    } else if (beakOut && !isExtending) {
      beak.setGoal(() -> SuperstructurePose.STOW.getBEAK_STOW_ANGLE());
      beakOut = false;
    }
  }
}
