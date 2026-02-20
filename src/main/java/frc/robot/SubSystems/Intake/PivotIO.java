package frc.robot.SubSystems.Intake;


import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface PivotIO {
    @AutoLog
  public static class PivotIOInputs {
    public boolean isMotorConnected = false;

    public Rotation2d position = new Rotation2d();
    public Rotation2d velocityUnitsPerSec = new Rotation2d();
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  public default void updateInputs(PivotIOInputs inputs) {

  }


  public default void setVoltage(double volts) {

  }

  /**
   * @param goalPosition The desired angular position for the pivot to be 
   *                     set to. Runs using internal MotionMagic
   */
  public default void setPosition(Rotation2d goalPosition) {

  }

  
  public default void stop() {

  }

  /**
   * Updates the gains of the feedback and feedforward
   * 
   * @param p
   * @param i
   * @param d
   * @param s
   * @param g
   * @param v
   * @param a
   */
  public default void setGains(double p, double i, double d, double s, double g, double v, double a) {

  }

  /**
   * Updates the gains of the profile. Note that profiled pid control is 
   * called "MotionMagic" by CTRE
   * 
   * @param maxVelocity The maximum achieveable velocity of the motor in 
   *                    meters per second
   * @param maxAcceleration The maximum achieveable acceleration of the motor 
   *                        in meters per second squared
   */
  public default void setMotionMagicConstraints(double maxVelocity, double maxAcceleration) {

  }


  
  public default void setBrakeMode(boolean enableBrake) {

  }

  /** Reset the relative encoder to 0 */
  public default void resetPosition() {
    
  }
}

