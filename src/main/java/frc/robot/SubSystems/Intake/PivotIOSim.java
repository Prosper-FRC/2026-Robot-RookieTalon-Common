package frc.robot.SubSystems.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.SubSystems.Intake.IntakeConstants;

public class PivotIOSim implements PivotIO {

  private final SingleJointedArmSim sim;

  private final PIDController pid =
      new PIDController(
          IntakeConstants.kPivotP,
          IntakeConstants.kPivotI,
          IntakeConstants.kPivotD);

  private final ArmFeedforward ff =
      new ArmFeedforward(
          IntakeConstants.kPivotS,
          IntakeConstants.kPivotG,
          IntakeConstants.kPivotV,
          IntakeConstants.kPivotA);

  private double appliedVolts = 0.0;
  private Rotation2d goalPosition = new Rotation2d();

  public PivotIOSim() {

    sim = new SingleJointedArmSim(
        edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(1),
        IntakeConstants.kPivotGearing,
        0.02, // moment of inertia (tune if needed)
        0.4,  // arm length meters (CHANGE to match CAD)
        IntakeConstants.kMinPivotPosition.getRadians(),
        IntakeConstants.kMaxPivotPosition.getRadians(),
        true,
        0.0
    );

    pid.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {

    sim.update(0.02); // 20ms loop

    inputs.position =
        Rotation2d.fromRadians(sim.getAngleRads());

    inputs.velocity =
        Rotation2d.fromRadians(sim.getVelocityRadPerSec());

    inputs.appliedVoltage = appliedVolts;
    inputs.supplyCurrent = sim.getCurrentDrawAmps();
    inputs.statorCurrent = sim.getCurrentDrawAmps();
    inputs.temperatureC = 25.0;
    inputs.isConnected = true;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12, 12);
    sim.setInputVoltage(appliedVolts);
  }

  @Override
  public void setPosition(Rotation2d position) {

    goalPosition = position;

    double pidOutput =
        pid.calculate(
            sim.getAngleRads(),
            goalPosition.getRadians());

    double ffOutput =
        ff.calculate(
            goalPosition.getRadians(),
            sim.getVelocityRadPerSec());

    appliedVolts =
        MathUtil.clamp(pidOutput + ffOutput, -12, 12);

    sim.setInputVoltage(appliedVolts);
  }

  @Override
  public void stop() {
    appliedVolts = 0;
    sim.setInputVoltage(0);
  }

  @Override
  public void resetPosition() {
    sim.setState(0, 0);
  }
}