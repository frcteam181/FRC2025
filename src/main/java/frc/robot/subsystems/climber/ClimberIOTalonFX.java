package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
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

public class ClimberIOTalonFX implements ClimberIO {

  // private final TalonFXS motor;
  private final TalonFX motor;

  // Config
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  // Status Signals
  private final StatusSignal<Angle> motorPositionRads;
  private final StatusSignal<AngularVelocity> motorVelocityRadsPerSec;
  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorSupplyCurrentAmps;
  private final StatusSignal<Current> motorTorqueCurrentAmps;
  private final StatusSignal<Temperature> motorTemp;

  // Cotrol Requests
  private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  // Connection debouncers
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5);

  public ClimberIOTalonFX() {

    motor = new TalonFX(17, "rio");

    // Configure motor
    config.Feedback.SensorToMechanismRatio = 1.0;
    // config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // config.CurrentLimits.SupplyCurrentLimit = 50;
    config.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -120.0;
    config.CurrentLimits.StatorCurrentLimit = 120.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(245);
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motor.getConfigurator().apply(config);
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> motor.setPosition(0.0));

    // Get and set status signals
    motorPositionRads = motor.getPosition();
    motorVelocityRadsPerSec = motor.getVelocity();
    motorAppliedVolts = motor.getMotorVoltage();
    motorSupplyCurrentAmps = motor.getSupplyCurrent();
    motorTorqueCurrentAmps = motor.getTorqueCurrent();
    motorTemp = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        motorPositionRads,
        motorVelocityRadsPerSec,
        motorAppliedVolts,
        motorSupplyCurrentAmps,
        motorTorqueCurrentAmps,
        motorTemp);
    motor.optimizeBusUtilization();

    // Register signals for refresh
    PhoenixUtil.registerSignals(
        false,
        motorPositionRads,
        motorVelocityRadsPerSec,
        motorAppliedVolts,
        motorSupplyCurrentAmps,
        motorTorqueCurrentAmps,
        motorTemp);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.data =
        new ClimberIOData(
            motorConnectedDebouncer.calculate(
                BaseStatusSignal.isAllGood(
                    motorPositionRads,
                    motorVelocityRadsPerSec,
                    motorAppliedVolts,
                    motorSupplyCurrentAmps,
                    motorTorqueCurrentAmps,
                    motorTemp)),
            motorPositionRads.getValue().in(Radians),
            motorVelocityRadsPerSec.getValue().in(RadiansPerSecond),
            motorAppliedVolts.getValueAsDouble(),
            motorSupplyCurrentAmps.getValueAsDouble(),
            motorTorqueCurrentAmps.getValueAsDouble(),
            motorTemp.getValueAsDouble());
  }

  @Override
  public void runVolts(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void runTorqueCurrent(double current) {
    motor.setControl(torqueCurrentFOC.withOutput(current));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    new Thread(
            () -> {
              tryUntilOk(
                  5,
                  () ->
                      motor.setNeutralMode(
                          enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast, 0.25));
            })
        .start();
  }
}
