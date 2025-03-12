package frc.robot.subsystems.superstructure.extender;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
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
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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

  // Hardware
  private final TalonFX talon;
  private final CANcoder encoder;

  // Config
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final CANcoderConfiguration canConfig = new CANcoderConfiguration();

  // Status Signals Talon Pivot
  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<AngularVelocity> pivotVelocity;
  private final StatusSignal<Voltage> pivotAppliedVolts;
  private final StatusSignal<Current> pivotSupplyCurrentAmps;
  private final StatusSignal<Current> pivotTorqueCurrentAmps;
  private final StatusSignal<Temperature> pivotTemp;

  // Cotrol Requests
  private final TorqueCurrentFOC motorTorqueCurrentFOC =
      new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final PositionTorqueCurrentFOC motorPositionTorqueCurrentFOC =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final VoltageOut motorVoltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  // Connected debouncers
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5);

  public ExtenderIOTalonFX() {
    talon = new TalonFX(4, "rio");
    encoder = new CANcoder(29, "rio");

    // Configure CANCoder
    canConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.65));
    canConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    canConfig.MagnetSensor.withMagnetOffset(Rotations.of(0.2));
    encoder.getConfigurator().apply(canConfig);

    // Configure motor
    config.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.SensorToMechanismRatio = 1.0;
    config.Feedback.RotorToSensorRatio = (5.0 * 5.0 * 3.0) * (22.0 / 10.0);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0 = new Slot0Configs().withKP(0.0).withKI(0.0).withKD(0.0);
    config.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -40;
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    talon.getConfigurator().apply(config);
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));

    // Get and set motor status signals
    pivotPosition = talon.getPosition();
    pivotVelocity = talon.getVelocity();
    pivotAppliedVolts = talon.getMotorVoltage();
    pivotSupplyCurrentAmps = talon.getSupplyCurrent();
    pivotTorqueCurrentAmps = talon.getTorqueCurrent();
    pivotTemp = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        pivotPosition,
        pivotVelocity,
        pivotAppliedVolts,
        pivotSupplyCurrentAmps,
        pivotTorqueCurrentAmps,
        pivotTemp);
    talon.optimizeBusUtilization();

    // Register signals for refresh
    PhoenixUtil.registerSignals(
        false,
        pivotPosition,
        pivotVelocity,
        pivotAppliedVolts,
        pivotSupplyCurrentAmps,
        pivotTorqueCurrentAmps,
        pivotTemp);
  }

  @Override
  public void updateInputs(ExtenderIOInputs inputs) {
    inputs.data =
        new ExtenderIOData(
            motorConnectedDebouncer.calculate(
                BaseStatusSignal.isAllGood(
                    pivotPosition,
                    pivotVelocity,
                    pivotAppliedVolts,
                    pivotSupplyCurrentAmps,
                    pivotTorqueCurrentAmps,
                    pivotTemp)),
            Rotation2d.fromRotations(pivotPosition.getValueAsDouble()),
            pivotVelocity.getValue().in(RadiansPerSecond),
            pivotAppliedVolts.getValue().in(Volts),
            pivotSupplyCurrentAmps.getValue().in(Amps),
            pivotTorqueCurrentAmps.getValue().in(Amps),
            pivotTemp.getValue().in(Celsius));
  }

  // Default pivot methods
  @Override
  public void runOpenLoop(double output) {
    talon.setControl(motorTorqueCurrentFOC.withOutput(output));
  }

  @Override
  public void runVolts(double volts) {
    talon.setControl(motorVoltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    talon.stopMotor();
  }

  @Override
  public void runPosition(Rotation2d position, double feedforward) {
    talon.setControl(
        motorPositionTorqueCurrentFOC
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
