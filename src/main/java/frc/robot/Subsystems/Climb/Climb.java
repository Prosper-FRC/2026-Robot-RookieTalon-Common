package frc.robot.Subsystems.Climb;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  /** Voltage setpoints for winch control */
  public enum ClimbVoltageGoal {
    kClimb,
    kDescend,
    kStop
  }

  private ClimbVoltageGoal voltageGoal = null;

  private final ClimbIO kMotor;
  private final ClimbIO.ClimbIOInputs kInputs;

  private final DutyCycleEncoderIO kAbsoluteEncoder;
  private final DutyCycleEncoderIO.DutyCycleEncoderIOInputs kAbsoluteEncoderInputs;

  private boolean isAbsoluteEncoderConnected = false;

  private final LoggedNetworkBoolean kDisableLimits =
    new LoggedNetworkBoolean("Climb/DisableLimits", false);

  public Climb(DutyCycleEncoderIO encoderIO, ClimbIO motorIO) {
    kMotor = motorIO;
    kInputs = new ClimbIO.ClimbIOInputs();

    kAbsoluteEncoder = encoderIO;
    kAbsoluteEncoderInputs = new DutyCycleEncoderIO.DutyCycleEncoderIOInputs();
  }

  @Override
  public void periodic() {
    // Update motor inputs
    kMotor.updateInputs(kInputs);
    // Log individual values instead of using processInputs
    Logger.recordOutput("Climb/Motor/Position", kInputs.position);
    Logger.recordOutput("Climb/Motor/Velocity", kInputs.velocityRotationsPerSec);
    Logger.recordOutput("Climb/Motor/AppliedVoltage", kInputs.appliedVoltage);
    Logger.recordOutput("Climb/Motor/SupplyCurrent", kInputs.supplyCurrentAmps);
    Logger.recordOutput("Climb/Motor/StatorCurrent", kInputs.statorCurrentAmps);
    Logger.recordOutput("Climb/Motor/Temperature", kInputs.temperatureCelsius);
    Logger.recordOutput("Climb/Motor/IsConnected", kInputs.isMotorConnected);

    // Update absolute encoder inputs
    kAbsoluteEncoder.updateInputs(kAbsoluteEncoderInputs);
    // Log encoder values
    Logger.recordOutput("Climb/AbsoluteEncoder/IsConnected", kAbsoluteEncoderInputs.isConnected);
    Logger.recordOutput("Climb/AbsoluteEncoder/FrequencyHz", kAbsoluteEncoderInputs.frequencyHz);
    Logger.recordOutput("Climb/AbsoluteEncoder/DutyCycle", kAbsoluteEncoderInputs.dutyCycleReading);

    isAbsoluteEncoderConnected = kAbsoluteEncoderInputs.isConnected;

    // Stop if robot is disabled
    if (!DriverStation.isEnabled()) {
      stop();
    }

    // Apply voltage goal if set
    if (voltageGoal != null) {
      double voltage = 0.0;
      switch (voltageGoal) {
        case kClimb:
          voltage = 12.0;
          break;
        case kDescend:
          voltage = -4.0;
          break;
        case kStop:
          voltage = 0.0;
          break;
      }
      setVoltage(voltage);
      Logger.recordOutput("Climb/VoltageGoal", voltageGoal.toString());
    } else {
      Logger.recordOutput("Climb/VoltageGoal", "NONE");
    }

    // Check soft limits if enabled
    if (!kDisableLimits.get()) {
      double currentPosition = getPosition().getRotations();
      if (currentPosition > ClimbConstants.kMaxPosition.getRotations()
          && kInputs.appliedVoltage > 0.0) {
        stop();
        Logger.recordOutput("Climb/LimitStatus", "Max limit hit");
      } else if (currentPosition < ClimbConstants.kMinPosition.getRotations()
          && kInputs.appliedVoltage < 0.0) {
        stop();
        Logger.recordOutput("Climb/LimitStatus", "Min limit hit");
      }
    }
  }

  public void setGoalVoltage(ClimbVoltageGoal desiredGoal) {
    voltageGoal = desiredGoal;
  }

  public void setVoltage(double voltage) {
    kMotor.setVoltage(voltage);
  }

  public void stop() {
    voltageGoal = null;
    kMotor.stop();
  }

  public void resetPosition() {
    kMotor.resetPosition();
  }

  /**
   * Get winch position in rotations from absolute encoder
   * Falls back to motor encoder if absolute encoder disconnected
   *
   * @return The position of the winch in rotations
   */
  @AutoLogOutput(key = "Climb/Position")
  public Rotation2d getPosition() {
    if (isAbsoluteEncoderConnected) {
      return Rotation2d.fromRotations(
        kAbsoluteEncoderInputs.dutyCycleReading)
        .minus(ClimbConstants.kPositionOffset);
    } else {
      Logger.recordOutput("Climb/Warning", "Using motor encoder - absolute encoder disconnected!");
      return kInputs.position;
    }
  }

  @AutoLogOutput(key = "Climb/Velocity")
  public Rotation2d getVelocity() {
    return kInputs.velocityRotationsPerSec;
  }

  @AutoLogOutput(key = "Climb/AbsoluteEncoderConnected")
  public boolean isAbsoluteEncoderConnected() {
    return isAbsoluteEncoderConnected;
  }

  @AutoLogOutput(key = "Climb/SupplyCurrent")
  public double getSupplyCurrentAmps() {
    return kInputs.supplyCurrentAmps;
  }
}