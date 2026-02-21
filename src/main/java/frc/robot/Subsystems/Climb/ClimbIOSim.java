package frc.robot.Subsystems.Climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Subsystems.Climb.ClimbConstants.ClimbGains;
import frc.robot.Subsystems.Climb.ClimbConstants.ClimbHardware;
import frc.robot.Subsystems.Climb.ClimbConstants.ClimbSimulationConfiguration;

public class ClimbIOSim implements ClimbIO {
  private final double kLoopPeriodSec;
  private final SingleJointedArmSim kPivot;
  private TrapezoidProfile kProfile;
  private final PIDController kFeedback;
  private ArmFeedforward kFeedforward;

  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  private double appliedVoltage = 0.0;
  private boolean feedbackNeedsReset = false;
  private boolean closedLoopControl = false;

  public ClimbIOSim(
      double loopPeriodSec,
      ClimbHardware hardware,
      ClimbSimulationConfiguration configuration,
      ClimbGains gains) {

    kLoopPeriodSec = loopPeriodSec;

    kPivot = new SingleJointedArmSim(
      configuration.motorType(),
      hardware.gearing(),
      configuration.momentOfInertiaJKgMetersSquared(),
      configuration.ligamentLengthMeters(),
      configuration.minPosition().getRadians(),
      configuration.maxPosition().getRadians(),
      configuration.simulateGravity(),
      configuration.initialPosition().getRadians(),
      configuration.measurementStdDevs(),
      configuration.measurementStdDevs());

    kProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
      gains.maxVelocityRotationsPerSec(),
      gains.maxAccelerationRotationsPerSecSquared()));

    kFeedback = new PIDController(gains.p(), gains.i(), gains.d());

    kFeedforward = new ArmFeedforward(gains.kS(), gains.kG(), gains.kV(), gains.kA());
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    kPivot.update(kLoopPeriodSec);

    inputs.isMotorConnected = true;

    inputs.position = Rotation2d.fromRadians(kPivot.getAngleRads());
    inputs.velocityRotationsPerSec = Rotation2d.fromRadians(kPivot.getVelocityRadPerSec());
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = 0.0;
    inputs.statorCurrentAmps = 0.0;
    inputs.temperatureCelsius = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    closedLoopControl = false;
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    kPivot.setInputVoltage(appliedVoltage);
  }

  @Override
  public void setPosition(Rotation2d goalPosition) {
    if (!closedLoopControl) {
      feedbackNeedsReset = true;
      closedLoopControl = true;
    }
    if (feedbackNeedsReset) {
      kFeedback.reset();
      setpoint = new TrapezoidProfile.State(kPivot.getAngleRads(), 0.0);
      feedbackNeedsReset = false;
    }
    goal = new TrapezoidProfile.State(goalPosition.getRadians(), 0.0);

    setpoint = kProfile.calculate(kLoopPeriodSec, setpoint, goal);

    double feedforwardEffort = kFeedforward.calculate(setpoint.position, setpoint.velocity);
    double feedbackEffort = kFeedback.calculate(kPivot.getAngleRads(), setpoint.position);

    String rootLogKey = getClass().getName() + "@" + Integer.toHexString(hashCode());

    Logger.recordOutput(rootLogKey + "/Feedback/FBEffort", feedbackEffort);
    Logger.recordOutput(rootLogKey + "/Feedback/FFEffort", feedforwardEffort);
    setVoltage(feedbackEffort + feedforwardEffort);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void setGains(double p, double i, double d, double s, double g, double v, double a) {
    kFeedback.setPID(p, i, d);
    kFeedforward = new ArmFeedforward(s, g, v, a);
  }

  @Override
  public void setMotionMagicConstraints(double maxVelocity, double maxAcceleration) {
    kProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
  }

  @Override
  public void resetPosition() {
    kPivot.setState(0.0, 0.0);
  }
}