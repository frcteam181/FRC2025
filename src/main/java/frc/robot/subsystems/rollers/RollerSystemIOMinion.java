package frc.robot.subsystems.rollers;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class RollerSystemIOMinion implements RollerSystemIO {
  private final TalonFXS talon;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;
  private final StatusSignal<Boolean> tempFault;

  // Single shot for voltage mode, robot loop will call continuously
  private final VoltageOut voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0);
  private final NeutralOut neutralOut = new NeutralOut();

  private final TalonFXSConfiguration config = new TalonFXSConfiguration();

  private final double reduction;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public RollerSystemIOMinion(
      int id, String bus, int currentLimitAmps, boolean invert, boolean brake, double reduction) {
    this.reduction = reduction;
    talon = new TalonFXS(id, bus);

    config.MotorOutput.Inverted =
        invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config));

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVoltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();
    tempFault = talon.getFault_DeviceTemp();

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                position,
                velocity,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius,
                tempFault));
    tryUntilOk(5, () -> talon.optimizeBusUtilization(0, 1.0));

    // Register signals for refresh
    PhoenixUtil.registerSignals(
        new CANBus(bus).isNetworkFD(),
        position,
        velocity,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        tempCelsius,
        tempFault);
  }

  @Override
  public void updateInputs(RollerSystemIOInputs inputs) {
    inputs.data =
        new RollerSystemIOData(
            Units.rotationsToRadians(position.getValueAsDouble()) / reduction,
            Units.rotationsToRadians(velocity.getValueAsDouble()) / reduction,
            appliedVoltage.getValueAsDouble(),
            supplyCurrent.getValueAsDouble(),
            torqueCurrent.getValueAsDouble(),
            tempCelsius.getValueAsDouble(),
            tempFault.getValue(),
            connectedDebouncer.calculate(
                BaseStatusSignal.isAllGood(
                    position,
                    velocity,
                    appliedVoltage,
                    supplyCurrent,
                    torqueCurrent,
                    tempCelsius,
                    tempFault)));
  }

  @Override
  public void runVolts(double volts) {
    talon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }

  @Override
  public void setCurrentLimit(double currentLimit) {
    new Thread(
        () -> {
          config.withCurrentLimits(config.CurrentLimits.withStatorCurrentLimit(currentLimit));
          tryUntilOk(5, () -> talon.getConfigurator().apply(config));
        });
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    new Thread(
            () ->
                tryUntilOk(
                    5,
                    () ->
                        talon.setNeutralMode(
                            enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast)))
        .start();
  }
}
