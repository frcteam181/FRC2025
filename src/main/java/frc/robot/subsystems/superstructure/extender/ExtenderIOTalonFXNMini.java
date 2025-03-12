package frc.robot.subsystems.superstructure.extender;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class ExtenderIOTalonFXNMini implements ExtenderIO {

  // Hardware
  private final TalonFX pivotTalon;
  private final TalonFXS rollerTalon;
  // private final CANcoder encoder;

  // Config
  private final TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
  private final TalonFXSConfiguration rollerConfig = new TalonFXSConfiguration();
  // private final CANcoderConfiguration canConfig = new CANcoderConfiguration();

  // Status Signals Talon Pivot
  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<AngularVelocity> pivotVelocity;
  private final StatusSignal<Voltage> pivotAppliedVolts;
  private final StatusSignal<Current> pivotSupplyCurrentAmps;
  private final StatusSignal<Current> pivotTorqueCurrentAmps;
  private final StatusSignal<Temperature> pivotTemp;

  // Status Signals Talon Roller
  private final StatusSignal<Angle> rollerPosition;
  private final StatusSignal<AngularVelocity> rollerVelocity;
  private final StatusSignal<Voltage> rollerAppliedVolts;
  private final StatusSignal<Current> rollerSupplyCurrentAmps;
  private final StatusSignal<Current> rollerTorqueCurrentAmps;
  private final StatusSignal<Temperature> rollerTemp;

  // Cotrol Requests
  private final TorqueCurrentFOC pivotTorqueCurrentFOC =
      new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final PositionTorqueCurrentFOC pivotPositionTorqueCurrentFOC =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final VoltageOut pivotVoltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  private final VoltageOut rollerVoltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  // Connected debouncers
  private final Debouncer pivotConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer rollerConnectedDebouncer = new Debouncer(0.5);

  public ExtenderIOTalonFXNMini() {
    pivotTalon = new TalonFX(4, "rio");
    rollerTalon = new TalonFXS(16, "rio");
    // encoder = new CANcoder(29, "rio");

    // Configure CANCoder
    // canConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.65));
    // canConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    // canConfig.MagnetSensor.withMagnetOffset(Rotations.of(0.2));
    // encoder.getConfigurator().apply(canConfig);

    // Configure pivot motor
    // config.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    // config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    pivotConfig.Feedback.SensorToMechanismRatio = (5.0 * 5.0 * 3.0) * (22.0 / 10.0);
    // config.Feedback.RotorToSensorRatio = 25.0;
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.Slot0 = new Slot0Configs().withKP(0.0).withKI(0.0).withKD(0.0);
    pivotConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    pivotConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
    pivotConfig.CurrentLimits.StatorCurrentLimit = 40;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    pivotTalon.getConfigurator().apply(pivotConfig);
    tryUntilOk(5, () -> pivotTalon.getConfigurator().apply(pivotConfig, 0.25));

    // Configure roller motor
    rollerConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    rollerConfig.ExternalFeedback.SensorToMechanismRatio = (5.0 * 5.0);
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = 50;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerTalon.getConfigurator().apply(rollerConfig);
    tryUntilOk(5, () -> rollerTalon.getConfigurator().apply(rollerConfig));

    // Get and set pivot status signals
    pivotPosition = pivotTalon.getPosition();
    pivotVelocity = pivotTalon.getVelocity();
    pivotAppliedVolts = pivotTalon.getMotorVoltage();
    pivotSupplyCurrentAmps = pivotTalon.getSupplyCurrent();
    pivotTorqueCurrentAmps = pivotTalon.getTorqueCurrent();
    pivotTemp = pivotTalon.getDeviceTemp();

    // Get and set pivot status signals
    rollerPosition = rollerTalon.getPosition();
    rollerVelocity = rollerTalon.getVelocity();
    rollerAppliedVolts = rollerTalon.getMotorVoltage();
    rollerSupplyCurrentAmps = rollerTalon.getSupplyCurrent();
    rollerTorqueCurrentAmps = rollerTalon.getTorqueCurrent();
    rollerTemp = rollerTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        pivotPosition,
        pivotVelocity,
        pivotAppliedVolts,
        pivotSupplyCurrentAmps,
        pivotTorqueCurrentAmps,
        pivotTemp,
        rollerPosition,
        rollerVelocity,
        rollerAppliedVolts,
        rollerSupplyCurrentAmps,
        rollerTorqueCurrentAmps,
        rollerTemp);
    pivotTalon.optimizeBusUtilization();
    rollerTalon.optimizeBusUtilization();

    // Register signals for refresh
    PhoenixUtil.registerSignals(
        false,
        pivotPosition,
        pivotVelocity,
        pivotAppliedVolts,
        pivotSupplyCurrentAmps,
        pivotTorqueCurrentAmps,
        pivotTemp,
        rollerPosition,
        rollerVelocity,
        rollerAppliedVolts,
        rollerSupplyCurrentAmps,
        rollerTorqueCurrentAmps,
        rollerTemp);
  }

  @Override
  public void updateInputs(ExtenderIOInputs inputs) {
    inputs.data =
        new ExtenderIOData(
            pivotConnectedDebouncer.calculate(
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
            pivotTemp.getValue().in(Celsius),
            rollerConnectedDebouncer.calculate(
                BaseStatusSignal.isAllGood(
                    rollerPosition,
                    rollerVelocity,
                    rollerAppliedVolts,
                    rollerSupplyCurrentAmps,
                    rollerTorqueCurrentAmps,
                    rollerTemp)),
            rollerPosition.getValueAsDouble(),
            rollerVelocity.getValue().in(RadiansPerSecond),
            rollerAppliedVolts.getValue().in(Volts),
            rollerSupplyCurrentAmps.getValue().in(Amps),
            rollerTorqueCurrentAmps.getValue().in(Amps),
            rollerTemp.getValue().in(Celsius));
  }

  // Default pivot methods
  @Override
  public void runPivotOpenLoop(double output) {
    pivotTalon.setControl(pivotTorqueCurrentFOC.withOutput(output));
  }

  @Override
  public void runPivotVolts(double volts) {
    pivotTalon.setControl(pivotVoltageRequest.withOutput(volts));
  }

  @Override
  public void stopPivot() {
    pivotTalon.stopMotor();
  }

  @Override
  public void runPivotPosition(Rotation2d position, double feedforward) {
    pivotTalon.setControl(
        pivotPositionTorqueCurrentFOC
            .withPosition(position.getRotations())
            .withFeedForward(feedforward));
  }

  @Override
  public void setPivotPID(double kP, double kI, double kD) {
    pivotConfig.Slot0.kP = kP;
    pivotConfig.Slot0.kI = kI;
    pivotConfig.Slot0.kD = kD;
    tryUntilOk(5, () -> pivotTalon.getConfigurator().apply(pivotConfig));
  }

  @Override
  public void setPivotBrakeMode(boolean enabled) {
    new Thread(
            () -> {
              pivotConfig.MotorOutput.NeutralMode =
                  enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
              tryUntilOk(5, () -> pivotTalon.getConfigurator().apply(pivotConfig));
            })
        .start();
  }

  // Default roller methods

  @Override
  public void runRollerVolts(double volts) {
    rollerTalon.setControl(rollerVoltageRequest.withOutput(volts));
  }

  @Override
  public void stopRoller() {
    rollerTalon.stopMotor();
  }

  @Override
  public void setRollerBrakeMode(boolean enabled) {
    new Thread(
            () -> {
              rollerConfig.MotorOutput.NeutralMode =
                  enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
              tryUntilOk(5, () -> rollerTalon.getConfigurator().apply(rollerConfig));
            })
        .start();
  }
}
