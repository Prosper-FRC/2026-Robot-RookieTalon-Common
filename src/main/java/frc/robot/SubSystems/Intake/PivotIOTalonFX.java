package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;

public class PivotIOTalonFX implements PivotIO {

  private final TalonFX motor =
      new TalonFX(IntakeConstants.kPivotMotorId);

  private final VoltageOut voltageRequest =
      new VoltageOut(0);

  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0);

  public PivotIOTalonFX() {

    TalonFXConfiguration config =
        new TalonFXConfiguration();

    config.MotorOutput.NeutralMode =
        IntakeConstants.kPivotNeutral;

    Slot0Configs slot0 = config.Slot0;
    slot0.kP = IntakeConstants.kPivotP;
    slot0.kI = IntakeConstants.kPivotI;
    slot0.kD = IntakeConstants.kPivotD;
    slot0.kS = IntakeConstants.kPivotS;
    slot0.kV = IntakeConstants.kPivotV;
    slot0.kA = IntakeConstants.kPivotA;
    slot0.kG = IntakeConstants.kPivotG;

    MotionMagicConfigs mm = config.MotionMagic;
    mm.MotionMagicCruiseVelocity =
        IntakeConstants.kCruiseVelocity;
    mm.MotionMagicAcceleration =
        IntakeConstants.kAcceleration;
    mm.MotionMagicJerk =
        IntakeConstants.kJerk;

    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.position =
        Rotation2d.fromRotations(
            motor.getPosition().getValueAsDouble());

    inputs.velocity =
        Rotation2d.fromRotations(
            motor.getVelocity().getValueAsDouble());

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
  public void setPosition(Rotation2d position) {
    motor.setControl(
        motionMagicRequest.withPosition(
            position.getRotations()));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void resetPosition() {
    motor.setPosition(0);
  }
}