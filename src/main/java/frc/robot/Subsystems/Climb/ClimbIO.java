package frc.robot.Subsystems.Climb;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;

/** Hardware interface for the climb subsystem motor */
public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    public boolean isMotorConnected = false;

    public Rotation2d position = new Rotation2d();
    public Rotation2d velocityRotationsPerSec = new Rotation2d();
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  /** Update inputs from hardware */
  public default void updateInputs(ClimbIOInputs inputs) {}

  /** Set motor voltage (-12 to 12 volts) */
  public default void setVoltage(double volts) {}

  /** Set motor to target position */
  public default void setPosition(Rotation2d goalPosition) {}

  /** Stop motor */
  public default void stop() {}

  /** Update PID and feedforward gains */
  public default void setGains(double p, double i, double d, double s, double g, double v, double a) {}

  /** Update motion magic constraints */
  public default void setMotionMagicConstraints(double maxVelocity, double maxAcceleration) {}

  /** Set brake or coast mode */
  public default void setBrakeMode(boolean enableBrake) {}

  /** Reset encoder position to zero */
  public default void resetPosition() {}
}