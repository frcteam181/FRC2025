package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure.Superstructure;
import java.util.function.DoubleSupplier;

public class SuperstructureCommands {

  private static final double DEADBAND = 0.1;

  private SuperstructureCommands() {}

  public static Command joystickMoveAll(
      Superstructure superstructure,
      DoubleSupplier elevatorSupplier,
      DoubleSupplier extenderSupplier,
      DoubleSupplier beakSupplier) {
    return Commands.run(
        () -> {
          superstructure.runElevatorOpenLoopVolt(elevatorSupplier.getAsDouble() * 2.5);
          superstructure.runExtenderOpenLoopVolt(extenderSupplier.getAsDouble() * 2.5);
          // superstructure.runBeakPivotOpenLoop(beakSupplier.getAsDouble() * 5.5);
          // superstructure.runExtenderRollerVolts(extenderSupplier.getAsDouble() * 5.5);
          // superstructure.runBeakIntakeOpenLoopVolt(beakSupplier.getAsDouble() * 5.5);
        },
        superstructure);
  }
}
