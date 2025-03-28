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

public class BeakIOTalonFX implements BeakIO {

  // private final TalonFXS motor;
  private final TalonFX motor;
  private final CANcoder encoder;

  // Config
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final CANcoderConfiguration canConfig = new CANcoderConfiguration();

  // Status Signals
  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<AngularVelocity> pivotVelocity;
  private final StatusSignal<Voltage> pivotAppliedVolts;
  private final StatusSignal<Current> pivotSupplyCurrentAmps;
  private final StatusSignal<Current> pivotTorqueCurrentAmps;
  private final StatusSignal<Temperature> pivotTemp;

  // Cotrol Requests
  private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentFOC =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  // Connection debouncers
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5);

  public BeakIOTalonFX() {
    // motor = new TalonFXS(3, "rio");
    motor = new TalonFX(3, "rio");
    encoder = new CANcoder(30, "rio");

    // Configure CANCoder
    canConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.3));
    canConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    canConfig.MagnetSensor.withMagnetOffset(Rotations.of(-0.1075));
    encoder.getConfigurator().apply(canConfig);

    // Configure motor
    config.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.SensorToMechanismRatio = 1.0;
    config.Feedback.RotorToSensorRatio = 5.0;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0 = new Slot0Configs().withKP(0.0).withKI(0.0).withKD(0.0);
    config.CurrentLimits.SupplyCurrentLimit = 50;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motor.getConfigurator().apply(config);
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    // Get and set status signals
    pivotPosition = motor.getPosition();
    pivotVelocity = motor.getVelocity();
    pivotAppliedVolts = motor.getMotorVoltage();
    pivotSupplyCurrentAmps = motor.getSupplyCurrent();
    pivotTorqueCurrentAmps = motor.getTorqueCurrent();
    pivotTemp = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        pivotPosition,
        pivotVelocity,
        pivotAppliedVolts,
        pivotSupplyCurrentAmps,
        pivotTorqueCurrentAmps,
        pivotTemp);
    motor.optimizeBusUtilization();

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
  public void updateInputs(PivotIOInputs inputs) {
    inputs.data =
        new PivotIOData(
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
