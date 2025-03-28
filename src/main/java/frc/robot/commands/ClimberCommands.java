package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;
import java.util.function.DoubleSupplier;

public class ClimberCommands {

  public ClimberCommands() {}

  public static Command runWithJoystick(Climber climber, DoubleSupplier climberVoltSupplier) {

    return Commands.run(
        () -> {
          climber.runVolts(climberVoltSupplier.getAsDouble() * 12);
        },
        climber);
  }
}
