package frc.robot.SubSystems.Intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;

public class IntakeIOSim implements IntakeIO {

  private final DCMotorSim motorSim;

  private double appliedVoltage = 0.0;

  public IntakeIOSim() {

    // Simulate 1 NEO motor
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNEO(1),
                0.001,     // moment of inertia (tune later)
                1.0        // gear ratio (adjust if geared)
            ),
            DCMotor.getNEO(1)
        );
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {

    // Update simulation by 20ms
    motorSim.update(0.02);

    inputs.positionRotations =
        motorSim.getAngularPositionRotations();

    inputs.velocityRPS =
        motorSim.getAngularVelocityRPM() / 60.0;

    inputs.appliedVoltage =
        appliedVoltage;

    inputs.supplyCurrent =
        motorSim.getCurrentDrawAmps();

    inputs.statorCurrent =
        motorSim.getCurrentDrawAmps();

    inputs.temperatureC =
        30.0; // fake temp

    inputs.isConnected = true;
  }

  @Override
  public void setVoltage(double volts) {

    appliedVoltage = volts;

    motorSim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void resetEncoder() {
    motorSim.setState(0.0, 0.0);
  }
}