package frc.robot.subsystems.superstructure.elevator;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
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

public class ElevatorIOTalonFX implements ElevatorIO {

  // Reduction
  // public static final double reduction = 1.0;

  // Motor Controllers
  private final TalonFX leader;
  private final TalonFX follower;

  // Encoder
  // private final CANcoder encoder;

  // Configs
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  // private final CANcoderConfiguration canConfig = new CANcoderConfiguration();

  private final TorqueCurrentFOC torqueCurrentRequest =
      new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0.0);

  // Satus signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;
  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> followerTorqueCurrent;
  private final StatusSignal<Current> followerSupplyCurrent;
  private final StatusSignal<Temperature> followerTemp;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public ElevatorIOTalonFX() {

    // Hardware
    leader = new TalonFX(14, "rio");
    follower = new TalonFX(5, "rio");
    follower.setControl(new Follower(leader.getDeviceID(), false));

    // encoder = new CANcoder(31, "rio");

    // Configure Encoder
    // canConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(1));
    // canConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    // canConfig.MagnetSensor.withMagnetOffset(Rotations.of(-0.355));
    // encoder.getConfigurator().apply(canConfig);

    // Configure motor
    // config.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    // config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    // config.Feedback.SensorToMechanismRatio = (5.0 * 4.0);
    // config.Feedback.RotorToSensorRatio = (5.0 * 5.0);
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0 = new Slot0Configs().withKP(0.0).withKI(0.0).withKD(0.0);
    config.Feedback.SensorToMechanismRatio = (5.0 * 5.0);
    config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    tryUntilOk(5, () -> leader.getConfigurator().apply(config, 0.25));

    position = leader.getPosition();
    velocity = leader.getVelocity();
    appliedVolts = leader.getMotorVoltage();
    torqueCurrent = leader.getTorqueCurrent();
    supplyCurrent = leader.getSupplyCurrent();
    temp = leader.getDeviceTemp();
    followerAppliedVolts = follower.getMotorVoltage();
    followerTorqueCurrent = follower.getTorqueCurrent();
    followerSupplyCurrent = follower.getSupplyCurrent();
    followerTemp = follower.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        position,
        velocity,
        appliedVolts,
        supplyCurrent,
        temp,
        followerAppliedVolts,
        followerTorqueCurrent,
        followerSupplyCurrent,
        followerTemp);
    torqueCurrent.setUpdateFrequency(1000);
    ParentDevice.optimizeBusUtilizationForAll(leader, follower);

    // Register signals for refresh
    PhoenixUtil.registerSignals(
        false,
        position,
        velocity,
        appliedVolts,
        torqueCurrent,
        supplyCurrent,
        temp,
        followerAppliedVolts,
        followerTorqueCurrent,
        followerSupplyCurrent,
        followerTemp);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.data =
        new ElevatorIOData(
            connectedDebouncer.calculate(
                BaseStatusSignal.isAllGood(
                    position, velocity, appliedVolts, torqueCurrent, supplyCurrent, temp)),
            connectedDebouncer.calculate(
                BaseStatusSignal.isAllGood(
                    followerAppliedVolts,
                    followerTorqueCurrent,
                    followerSupplyCurrent,
                    followerTemp)),
            Units.rotationsToRadians(position.getValueAsDouble()),
            Units.rotationsToRadians(velocity.getValueAsDouble()),
            appliedVolts.getValueAsDouble(),
            torqueCurrent.getValueAsDouble(),
            supplyCurrent.getValueAsDouble(),
            temp.getValueAsDouble(),
            followerAppliedVolts.getValueAsDouble(),
            followerTorqueCurrent.getValueAsDouble(),
            followerSupplyCurrent.getValueAsDouble(),
            followerTemp.getValueAsDouble());
  }

  @Override
  public void runOpenLoop(double output) {
    leader.setControl(torqueCurrentRequest.withOutput(output));
  }

  @Override
  public void runVolts(double volts) {
    leader.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void runPosition(double positionRad, double feedforward) {
    leader.setControl(
        positionTorqueCurrentRequest
            .withPosition(Units.radiansToRotations(positionRad))
            .withFeedForward(feedforward));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    tryUntilOk(5, () -> leader.getConfigurator().apply(config));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    new Thread(
            () -> {
              config.MotorOutput.NeutralMode =
                  enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
              tryUntilOk(5, () -> leader.getConfigurator().apply(config));
            })
        .start();
  }
}
