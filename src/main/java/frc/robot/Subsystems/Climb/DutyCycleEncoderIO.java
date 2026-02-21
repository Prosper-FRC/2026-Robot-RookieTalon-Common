package frc.robot.Subsystems.Climb;

import org.littletonrobotics.junction.AutoLog;

/** Hardware interface for the absolute encoder */
public interface DutyCycleEncoderIO {
  @AutoLog
  public static class DutyCycleEncoderIOInputs {
    public boolean isConnected = false;
    public int frequencyHz = 0;
    public double dutyCycleReading = 0.0;
  }

  /** Update inputs from hardware */
  public default void updateInputs(DutyCycleEncoderIOInputs inputs) {}
}