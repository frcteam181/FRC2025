package frc.robot.subsystems.superstructure.extender;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class ExtenderIOTalonFX implements ExtenderIO {
  public static final double reduction = (22.0 / 14.0);

  // Hardware
  private final TalonFX talon;
  private final CANcoder encoder;

  // Config
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final CANcoderConfiguration canConfig = new CANcoderConfiguration();

  // Status Signals
  private final StatusSignal<Angle> internalPosition;
  private final StatusSignal<AngularVelocity> internalVelocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> torqueCurrentAmps;
  private final StatusSignal<Temperature> temp;

  // Cotrol Requests
  private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentFOC =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  // Connected debouncers
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5);

  public ExtenderIOTalonFX() {
    talon = new TalonFX(4, "rio");
    encoder = new CANcoder(29, "rio");

    // Configure motor
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0 = new Slot0Configs().withKP(0.0).withKI(0.0).withKD(0.0);
    config.Feedback.SensorToMechanismRatio = reduction;
    config.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -40;
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));

    // Configure CANCoder
    canConfig.MagnetSensor.MagnetOffset = -0.0209;
    canConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoder.getConfigurator().apply(canConfig);

    // Get and set status signals
    internalPosition = encoder.getAbsolutePosition();
    internalVelocity = encoder.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    supplyCurrentAmps = talon.getSupplyCurrent();
    torqueCurrentAmps = talon.getTorqueCurrent();
    temp = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        internalPosition,
        internalVelocity,
        appliedVolts,
        supplyCurrentAmps,
        torqueCurrentAmps,
        temp);
    talon.optimizeBusUtilization();

    // Register signals for refresh
    PhoenixUtil.registerSignals(
        false,
        internalPosition,
        internalVelocity,
        appliedVolts,
        supplyCurrentAmps,
        torqueCurrentAmps,
        temp);
  }

  @Override
  public void updateInputs(ExtenderIOInputs inputs) {
    inputs.data =
        new ExtenderIOData(
            motorConnectedDebouncer.calculate(
                BaseStatusSignal.isAllGood(
                    internalPosition,
                    internalVelocity,
                    appliedVolts,
                    supplyCurrentAmps,
                    torqueCurrentAmps,
                    temp)),
            Rotation2d.fromRotations(internalPosition.getValueAsDouble()),
            internalVelocity.getValue().in(RadiansPerSecond),
            appliedVolts.getValue().in(Volts),
            supplyCurrentAmps.getValue().in(Amps),
            torqueCurrentAmps.getValue().in(Amps),
            temp.getValue().in(Celsius));
  }

  @Override
  public void runOpenLoop(double output) {
    talon.setControl(torqueCurrentFOC.withOutput(output));
  }

  @Override
  public void runVolts(double volts) {
    talon.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    talon.stopMotor();
  }

  @Override
  public void runPosition(Rotation2d position, double feedforward) {
    talon.setControl(
        positionTorqueCurrentFOC
            .withPosition(position.getRotations())
            .withFeedForward(feedforward));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    new Thread(
            () -> {
              config.MotorOutput.NeutralMode =
                  enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
              tryUntilOk(5, () -> talon.getConfigurator().apply(config));
            })
        .start();
  }
}
