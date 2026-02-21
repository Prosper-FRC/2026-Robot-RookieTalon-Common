package frc.robot.Subsystems.Climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Subsystems.Climb.ClimbConstants.ClimbGains;
import frc.robot.Subsystems.Climb.ClimbConstants.ClimbHardware;
import frc.robot.Subsystems.Climb.ClimbConstants.ClimbTalonFXConfiguration;

public class ClimbIOTalonFX implements ClimbIO {
  private final TalonFX kMotor;
  private TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

  private StatusSignal<Angle> positionRotations;
  private StatusSignal<AngularVelocity> velocityRotationsPerSec;
  private StatusSignal<Voltage> appliedVolts;
  private StatusSignal<Current> supplyCurrentAmps;
  private StatusSignal<Current> statorCurrentAmps;
  private StatusSignal<Temperature> temperatureCelsius;

  private final VoltageOut kVoltageControl = new VoltageOut(0.0);
  private final MotionMagicVoltage kPositionControl = new MotionMagicVoltage(0.0);

  public ClimbIOTalonFX(
      String canbus,
      ClimbHardware hardware,
      ClimbTalonFXConfiguration configuration,
      ClimbGains gains,
      double statusSignalUpdateFrequency) {

    kMotor = new TalonFX(hardware.motorId(), canbus);

    // Configure PID and feedforward gains
    motorConfiguration.Slot0.kP = gains.p();
    motorConfiguration.Slot0.kI = gains.i();
    motorConfiguration.Slot0.kD = gains.d();
    motorConfiguration.Slot0.kS = gains.kS();
    motorConfiguration.Slot0.kV = gains.kV();
    motorConfiguration.Slot0.kA = gains.kA();
    motorConfiguration.Slot0.kG = gains.kG();

    // Configure motion magic constraints
    motorConfiguration.MotionMagic.MotionMagicCruiseVelocity = 
      gains.maxVelocityRotationsPerSec();
    motorConfiguration.MotionMagic.MotionMagicAcceleration = 
      gains.maxAccelerationRotationsPerSecSquared();
    motorConfiguration.MotionMagic.MotionMagicJerk = 
      gains.jerkRotationsPerSecCubed();

    // Configure current limits
    motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = 
      configuration.enableSupplyCurrentLimit();
    motorConfiguration.CurrentLimits.SupplyCurrentLimit = 
      configuration.supplyCurrentLimitAmps();
    motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = 
      configuration.enableStatorCurrentLimit();
    motorConfiguration.CurrentLimits.StatorCurrentLimit = 
      configuration.statorCurrentLimitAmps();

    // Configure voltage limits
    motorConfiguration.Voltage.PeakForwardVoltage = 
      configuration.peakForwardVoltage();
    motorConfiguration.Voltage.PeakReverseVoltage = 
      configuration.peakReverseVoltage();

    // Configure motor output
    motorConfiguration.MotorOutput.NeutralMode = configuration.neutralMode();
    motorConfiguration.MotorOutput.Inverted =
      configuration.invert()
        ? InvertedValue.CounterClockwise_Positive
        : InvertedValue.Clockwise_Positive;

    // Reset position on startup
    kMotor.setPosition(0.0);

    // Configure feedback sensor
    motorConfiguration.Feedback.SensorToMechanismRatio = hardware.gearing();
    motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    kMotor.getConfigurator().apply(motorConfiguration, 1.0);

    // Get status signals
    positionRotations = kMotor.getPosition();
    velocityRotationsPerSec = kMotor.getVelocity();
    appliedVolts = kMotor.getMotorVoltage();
    supplyCurrentAmps = kMotor.getSupplyCurrent();
    statorCurrentAmps = kMotor.getStatorCurrent();
    temperatureCelsius = kMotor.getDeviceTemp();

    // Set update frequencies
    BaseStatusSignal.setUpdateFrequencyForAll(
      statusSignalUpdateFrequency,
      positionRotations,
      velocityRotationsPerSec,
      appliedVolts,
      supplyCurrentAmps,
      statorCurrentAmps,
      temperatureCelsius);

    kMotor.optimizeBusUtilization(0.0, 1.0);
  }

  public ClimbIOTalonFX(
      ClimbHardware hardware,
      ClimbTalonFXConfiguration configuration,
      ClimbGains gains,
      double statusSignalUpdateFrequency) {
    this("rio", hardware, configuration, gains, statusSignalUpdateFrequency);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.isMotorConnected = BaseStatusSignal.refreshAll(
      positionRotations,
      velocityRotationsPerSec,
      appliedVolts,
      supplyCurrentAmps,
      statorCurrentAmps,
      temperatureCelsius).isOK();

    inputs.position = Rotation2d.fromRotations(positionRotations.getValueAsDouble());
    inputs.velocityRotationsPerSec = Rotation2d.fromRotations(velocityRotationsPerSec.getValueAsDouble());
    inputs.appliedVoltage = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    kMotor.setControl(kVoltageControl.withOutput(volts));
  }

  @Override
  public void setPosition(Rotation2d goalPosition) {
    kMotor.setControl(
      kPositionControl.withPosition(goalPosition.getRotations()).withSlot(0));
  }

  @Override
  public void stop() {
    kMotor.setControl(new NeutralOut());
  }

  @Override
  public void resetPosition() {
    kMotor.setPosition(0.0);
  }

  @Override
  public void setGains(double p, double i, double d, double s, double g, double v, double a) {
    var slotConfiguration = new Slot0Configs();

    slotConfiguration.kP = p;
    slotConfiguration.kI = i;
    slotConfiguration.kD = d;
    slotConfiguration.kS = s;
    slotConfiguration.kG = g;
    slotConfiguration.kV = v;
    slotConfiguration.kA = a;

    kMotor.getConfigurator().apply(slotConfiguration);
  }

  @Override
  public void setMotionMagicConstraints(double maxVelocity, double maxAcceleration) {
    var motionMagicConfiguration = new MotionMagicConfigs();

    motionMagicConfiguration.MotionMagicCruiseVelocity = maxVelocity;
    motionMagicConfiguration.MotionMagicAcceleration = maxAcceleration;
    motionMagicConfiguration.MotionMagicJerk = 10.0 * maxAcceleration;

    kMotor.getConfigurator().apply(motionMagicConfiguration);
  }

  @Override
  public void setBrakeMode(boolean enableBrake) {
    kMotor.setNeutralMode(enableBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}