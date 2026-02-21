package frc.robot.Subsystems.Climb;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/** Constants for the winch climber subsystem */
public class ClimbConstants {
  
  public record ClimbHardware(
    int motorId,
    double gearing
  ) {}

  public record ClimbGains(
    double p,
    double i,
    double d,
    double maxVelocityRotationsPerSec,
    double maxAccelerationRotationsPerSecSquared,
    double jerkRotationsPerSecCubed,
    double kS,
    double kV,
    double kA,
    double kG
  ) {}

  public record ClimbTalonFXConfiguration(
    boolean invert,
    boolean enableStatorCurrentLimit,
    boolean enableSupplyCurrentLimit,
    double statorCurrentLimitAmps,
    double supplyCurrentLimitAmps,
    double peakForwardVoltage,
    double peakReverseVoltage,
    NeutralModeValue neutralMode
  ) {}

  public record ClimbSimulationConfiguration(
    DCMotor motorType,
    double momentOfInertiaJKgMetersSquared,
    double ligamentLengthMeters,
    Rotation2d minPosition,
    Rotation2d maxPosition,
    boolean simulateGravity,
    Rotation2d initialPosition,
    double measurementStdDevs
  ) {}

  public record DutyCycleConfiguration(
    int encoderChannel,
    int connectedFrequencyThresholdHz,
    double minimumDutyCycleRange,
    double maximumDutyCycleRange
  ) {}

  // Winch position limits in rotations
  public static final Rotation2d kMinPosition = Rotation2d.fromRotations(0.0);
  public static final Rotation2d kMaxPosition = Rotation2d.fromRotations(50.0);

  // Offset for absolute encoder (duty cycle to rotations)
  public static final Rotation2d kPositionOffset = Rotation2d.fromRotations(0.0);

  // Motor configuration
  public static final double kGearing = 50.0;
  public static final double kStatusSignalUpdateFrequency = 20.0;

  public static final ClimbHardware kMotorHardware = new ClimbHardware(
    40,        // CAN ID - TUNE THIS
    kGearing
  );

  public static final ClimbGains kMotorGains = new ClimbGains(
    5.0,       // kP
    0.0,       // kI
    0.1,       // kD
    2.0,       // maxVelocity
    2.0,       // maxAcceleration
    0.0,       // jerk
    0.0,       // kS
    0.0,       // kV
    0.0,       // kA
    0.5        // kG (gravity/load)
  );

  public static final ClimbTalonFXConfiguration kMotorConfiguration =
    new ClimbTalonFXConfiguration(
      false,                    // Don't invert
      true,                     // Enable stator current limit
      true,                     // Enable supply current limit
      80.0,                     // Stator limit (amps)
      60.0,                     // Supply limit (amps)
      12.0,                     // Peak forward voltage
      -8.0,                     // Peak reverse voltage
      NeutralModeValue.Brake    // Brake mode to hold position
    );

  public static final ClimbSimulationConfiguration kSimulationConfiguration =
    new ClimbSimulationConfiguration(
      DCMotor.getKrakenX60(1),
      0.05,
      Units.inchesToMeters(2.0),
      kMinPosition,
      kMaxPosition,
      false,
      Rotation2d.fromRotations(0.0),
      0.002
    );

  public static final DutyCycleConfiguration kDutyCycleConfiguration =
    new DutyCycleConfiguration(
      9,                        // PWM channel - TUNE THIS
      960,                      // Connected frequency threshold
      0.0,                      // Min duty cycle
      1.0                       // Max duty cycle
    );
}