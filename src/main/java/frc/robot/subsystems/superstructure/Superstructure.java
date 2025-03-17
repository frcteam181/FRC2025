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
    extender.startEjectTimer(); // -----------------------------------
    extender.setGripperGoal(GripperGoal.EJECT); // Swapped the order of these two lines
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

  // public void ELEVATOR_TO_STOW() {
  //   setGoal(SuperstructureState.STOW);
  // }

  // public void ELEVATOR_TO_ALGAE_L2_INTAKE() {
  //   setGoal(SuperstructureState.ALGAE_L2_INTAKE);
  // }

  // public void ELEVATOR_TO_ALGAE_L3_INTAKE() {
  //   setGoal(SuperstructureState.ALGAE_L3_INTAKE);
  // }

  // public void ELEVATOR_TO_NET() {
  //   setGoal(SuperstructureState.NET);
  // }

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

  public void calculateState() {
    if (pose != previousPose || currentPose == SuperstructurePose.START) {
      switch (pose) {
          /* ----------------------------------------START---------------------------------------- */
        case START:
          if (!elevator.isHomed()) {
            elevator.homingSequence();
          } else {
            setPose(SuperstructurePose.STOW);
          }
          break;
          /* -------------------------------------AUTO_START---------------------------------------- */
        case AUTO_START:
          break;
          /* -------------------------------------STOW---------------------------------------- */
        case STOW:
          // if (elevator.)
          elevator.setGoal(() -> STOW.getHEIGHT());
          // beak.setGoal(
          //     () -> STOW.getBeakAngle()); // Change this so beak has the stowed angle and the
          // extended
          // // angle
          // if (shouldExtend) {
          //   extender.setPivotGoal(() -> STOW.getEXT_ANGLE());
          // }
          break;
          /* -------------------------------------L1 CORAL---------------------------------------- */
        case L1_CORAL:
          elevator.setGoal(() -> L1_CORAL.getHEIGHT());
          // beak.setGoal(() -> L1_CORAL.getBeakAngle());
          // if (shouldExtend) {
          //   extender.setPivotGoal(() -> L1_CORAL.getEXT_ANGLE());
          //   break;
          // }
          // extender.setPivotGoal(() -> STOW.getEXT_ANGLE());
          break;
          /* -------------------------------------L2 CORAL---------------------------------------- */
        case L2_CORAL:
          elevator.setGoal(() -> L2_CORAL.getHEIGHT());
          // beak.setGoal(() -> L2_CORAL.getBeakAngle());
          // if (shouldExtend) {
          //   extender.setPivotGoal(() -> L2_CORAL.getEXT_ANGLE());
          //   break;
          // }
          // extender.setPivotGoal(() -> STOW.getEXT_ANGLE());
          break;
          /* -------------------------------------L3 CORAL---------------------------------------- */
        case L3_CORAL:
          elevator.setGoal(() -> L3_CORAL.getHEIGHT());
          // beak.setGoal(() -> L3_CORAL.getBeakAngle());
          // if (shouldExtend) {
          //   extender.setPivotGoal(() -> L3_CORAL.getEXT_ANGLE());
          //   break;
          // }
          // extender.setPivotGoal(() -> STOW.getEXT_ANGLE());
          break;
          /* -------------------------------------L4 CORAL---------------------------------------- */
        case L4_CORAL:
          elevator.setGoal(() -> L4_CORAL.getHEIGHT());
          // beak.setGoal(() -> L4_CORAL.getBeakAngle());
          // if (shouldExtend) {
          //   extender.setPivotGoal(() -> L4_CORAL.getEXT_ANGLE());
          //   break;
          // }
          // extender.setPivotGoal(() -> STOW.getEXT_ANGLE());
          break;
          /* -------------------------------------L1 CORAL EJECT---------------------------------------- */
        case L1_CORAL_EJECT:
          elevator.setGoal(() -> L1_CORAL.getHEIGHT());
          // beak.setGoal(() -> L1_CORAL.getBeakAngle());
          // if (shouldExtend) {
          //   extender.setPivotGoal(() -> L1_CORAL.getEXT_ANGLE());
          //   break;
          // }
          // extender.setPivotGoal(() -> STOW.getEXT_ANGLE());
          break;
          /* -------------------------------------L2 CORAL EJECT---------------------------------------- */
        case L2_CORAL_EJECT:
          elevator.setGoal(() -> L2_CORAL.getHEIGHT());
          // beak.setGoal(() -> L2_CORAL.getBeakAngle());
          // if (shouldExtend) {
          //   extender.setPivotGoal(() -> L2_CORAL.getEXT_ANGLE());
          //   break;
          // }
          // extender.setPivotGoal(() -> STOW.getEXT_ANGLE());
          break;
          /* -------------------------------------L3 CORAL EJECT---------------------------------------- */
        case L3_CORAL_EJECT:
          elevator.setGoal(() -> L3_CORAL.getHEIGHT());
          // beak.setGoal(() -> L3_CORAL.getBeakAngle());
          // if (shouldExtend) {
          //   extender.setPivotGoal(() -> L3_CORAL.getEXT_ANGLE());
          //   break;
          // }
          // extender.setPivotGoal(() -> STOW.getEXT_ANGLE());
          break;
          /* -------------------------------------L4 CORAL EJECT---------------------------------------- */
        case L4_CORAL_EJECT:
          elevator.setGoal(() -> L4_CORAL.getHEIGHT());
          // beak.setGoal(() -> L4_CORAL.getBeakAngle());
          // if (shouldExtend) {
          //   extender.setPivotGoal(() -> L4_CORAL.getEXT_ANGLE());
          //   break;
          // }
          // extender.setPivotGoal(() -> STOW.getEXT_ANGLE());
          break;
          /* -------------------------------------ALGAE FLOOR INTAKE---------------------------------------- */
        case ALGAE_L2_INTAKE:
          elevator.setGoal(() -> ALGAE_L2_INTAKE.getHEIGHT());
          // beak.setGoal(() -> ALGAE_L2_INTAKE.getBeakAngle());
          // if (shouldExtend) {
          //   extender.setPivotGoal(() -> ALGAE_L2_INTAKE.getEXT_ANGLE());
          //   break;
          // }
          // extender.setPivotGoal(() -> STOW.getEXT_ANGLE());
          break;
          /* -------------------------------------ALGAE L2 INTAKE---------------------------------------- */
        case ALGAE_L3_INTAKE:
          elevator.setGoal(() -> ALGAE_L3_INTAKE.getHEIGHT());
          // beak.setGoal(() -> ALGAE_L3_INTAKE.getBeakAngle());
          // if (shouldExtend) {
          //   extender.setPivotGoal(() -> ALGAE_L3_INTAKE.getEXT_ANGLE());
          //   break;
          // }
          // extender.setPivotGoal(() -> STOW.getEXT_ANGLE());
          break;
          /* -------------------------------------NET---------------------------------------- */
        case NET:
          elevator.setGoal(() -> NET.getHEIGHT());
          // beak.setGoal(() -> NET.getBeakAngle());
          // if (shouldExtend) {
          //   extender.setPivotGoal(() -> NET.getEXT_ANGLE());
          //   break;
          // }
          // extender.setPivotGoal(() -> STOW.getEXT_ANGLE());
          break;

        default:
          break;
      }
      previousPose = pose;
    }
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

  public void calculateBeakAngle() {
    if (isExtending && !beakOut) {
      beak.setGoal(() -> getPose().getBEAK_OUT_ANGLE());
    } else if (beakOut && !isExtending) {
      beak.setGoal(() -> SuperstructurePose.STOW.getBEAK_STOW_ANGLE());
    }
  }
}
