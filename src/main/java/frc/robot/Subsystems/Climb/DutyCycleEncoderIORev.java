package frc.robot.Subsystems.Climb;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Subsystems.Climb.ClimbConstants.DutyCycleConfiguration;

/**
 * Hardware implementation for REV Through Bore encoder
 * Set up as a duty cycle encoder (absolute encoder)
 * 
 * Related documentation:
 * https://docs.revrobotics.com/rev-crossover-products/sensors/tbe
 */
public class DutyCycleEncoderIORev implements DutyCycleEncoderIO {
  private final DutyCycleEncoder kEncoder;

  public DutyCycleEncoderIORev(DutyCycleConfiguration configuration) {
    kEncoder = new DutyCycleEncoder(configuration.encoderChannel());

    kEncoder.setConnectedFrequencyThreshold(configuration.connectedFrequencyThresholdHz());
    kEncoder.setDutyCycleRange(
      configuration.minimumDutyCycleRange(),
      configuration.maximumDutyCycleRange());
  }

  @Override
  public void updateInputs(DutyCycleEncoderIOInputs inputs) {
    inputs.isConnected = kEncoder.isConnected();
    inputs.frequencyHz = kEncoder.getFrequency();
    inputs.dutyCycleReading = kEncoder.get();
  }
}