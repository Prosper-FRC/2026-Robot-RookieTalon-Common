package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  class IntakeInputs {
    public boolean isConnected = false;
    public double positionRotations = 0.0;
    public double velocityRPS = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrent = 0.0;
    public double statorCurrent = 0.0;
    public double temperatureC = 0.0;
  }

  default void updateInputs(IntakeInputs inputs) {
    
  }

  default void setVoltage(double volts) {

  }

  default void stop() {

  }

  default void resetEncoder() {

  }
}