package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX motor =
      new TalonFX(IntakeConstants.kRollerMotorId);

  private final VoltageOut voltageRequest =
      new VoltageOut(0);

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.positionRotations =
        motor.getPosition().getValueAsDouble();
    inputs.velocityRPS =
        motor.getVelocity().getValueAsDouble();
    inputs.appliedVoltage =
        motor.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrent =
        motor.getSupplyCurrent().getValueAsDouble();
    inputs.statorCurrent =
        motor.getStatorCurrent().getValueAsDouble();
    inputs.temperatureC =
        motor.getDeviceTemp().getValueAsDouble();
    inputs.isConnected = motor.isConnected();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void resetEncoder() {
    motor.setPosition(0);
  }
}