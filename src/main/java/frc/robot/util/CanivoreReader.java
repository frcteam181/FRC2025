package frc.robot.util;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Constants;
import java.util.Optional;

public class CanivoreReader {
  private final CANBus canBus;
  private final Notifier notifier;
  private Optional<CANBusStatus> status = Optional.empty();

  public CanivoreReader(String canBusName) {
    canBus = new CANBus(canBusName);
    notifier =
        new Notifier(
            () -> {
              var statusTemp = Optional.of(canBus.getStatus());
              synchronized (this) {
                status = statusTemp;
              }
            });
    notifier.startPeriodic(Constants.loopPeriodSecs);
  }

  public synchronized Optional<CANBusStatus> getStatus() {
    return status;
  }
}
