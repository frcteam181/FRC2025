package frc.robot.subsystems.superstructure.beak;

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
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
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

public class BeakIOMinion implements BeakIO {

  private final TalonFXS motor;
  private final CANcoder encoder;

  // Config
  private final TalonFXSConfiguration config = new TalonFXSConfiguration();
  private final CANcoderConfiguration canConfig = new CANcoderConfiguration();

  // Status Signals
  private final StatusSignal<Angle> absPosition;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;

  // Cotrol Requests
  private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentFOC =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  // Connection debouncers
  private final Debouncer motorConnectedDebounce = new Debouncer(0.5);
  private final Debouncer encoderConnectedDebounce = new Debouncer(0.5);

  public BeakIOMinion() {
    motor = new TalonFXS(3, "rio");
    encoder = new CANcoder(30, "rio");

    // Configure CANCoder
    config.ExternalFeedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    config.ExternalFeedback.ExternalFeedbackSensorSource =
        ExternalFeedbackSensorSourceValue.FusedCANcoder;
    canConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.3));
    canConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    canConfig.MagnetSensor.withMagnetOffset(Rotations.of(-0.111));
    encoder.getConfigurator().apply(canConfig);

    // Configure motor
    config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    config.ExternalFeedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    config.ExternalFeedback.ExternalFeedbackSensorSource =
        ExternalFeedbackSensorSourceValue.FusedCANcoder;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0 = new Slot0Configs().withKP(0.0).withKI(0.0).withKD(0.0);
    config.ExternalFeedback.SensorToMechanismRatio = 1.0;
    config.ExternalFeedback.RotorToSensorRatio = 5.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    // Get and set status signals
    absPosition = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    torqueCurrent = motor.getTorqueCurrent();
    supplyCurrent = motor.getSupplyCurrent();
    temp = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, absPosition, velocity, appliedVolts, torqueCurrent, supplyCurrent, temp);
    motor.optimizeBusUtilization();

    // Register signals for refresh
    PhoenixUtil.registerSignals(
        false, absPosition, velocity, appliedVolts, torqueCurrent, supplyCurrent, temp);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.data =
        new PivotIOData(
            motorConnectedDebounce.calculate(BaseStatusSignal.isAllGood(appliedVolts, temp)),
            encoderConnectedDebounce.calculate(BaseStatusSignal.isAllGood(absPosition, velocity)),
            Rotation2d.fromRotations(absPosition.getValueAsDouble()),
            velocity.getValue().in(RadiansPerSecond),
            appliedVolts.getValue().in(Volts),
            torqueCurrent.getValue().in(Amps),
            supplyCurrent.getValue().in(Amps),
            temp.getValue().in(Celsius));
  }

  @Override
  public void runOpenLoop(double output) {
    motor.setControl(torqueCurrentFOC.withOutput(output));
  }

  @Override
  public void runVolts(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void runPosition(Rotation2d position, double feedforward) {
    motor.setControl(
        positionTorqueCurrentFOC
            .withPosition(position.getRotations())
            .withFeedForward(feedforward));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    new Thread(
            () -> {
              config.MotorOutput.NeutralMode =
                  enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
              tryUntilOk(5, () -> motor.getConfigurator().apply(config));
            })
        .start();
  }
}
