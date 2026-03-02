package frc.robot.SubSystems.Intake;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;

public interface PivotIO {

  @AutoLog
  class PivotIOInputs {
    public boolean isConnected = false;
    public Rotation2d position = new Rotation2d();
    public Rotation2d velocity = new Rotation2d();
    public double appliedVoltage = 0.0;
    public double supplyCurrent = 0.0;
    public double statorCurrent = 0.0;
    public double temperatureC = 0.0;
  }

  default void updateInputs(PivotIOInputs inputs) {

  }

  default void setVoltage(double volts) {

  }

  default void setPosition(Rotation2d position) {

  }

  default void stop() {

  }

  default void resetPosition() {
    
  }
}