package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Subsystems.Shooter.ShooterConstants.FlywheelGains;
import frc.robot.Subsystems.Shooter.ShooterConstants.HooderGains;
import frc.robot.Subsystems.Shooter.ShooterConstants.IndexerGains;

// ADD PID CONTROLLERS FOR FLYWHEEL AND INDEXER
public class ShooterIOSim implements ShooterIO {
    private static final double kLoopPeriodSec = 0.02;

    private final FlywheelSim kFlywheel;
    private final DCMotorSim kIndexer;
    private final SingleJointedArmSim kHooder;

    @AutoLogOutput(key = "Shooter/HooderPositionRadiansGoal")
    private double kHooderPositionRadiansGoal = 0.0;

    @AutoLogOutput(key = "Shooter/IndexerVelocityGoal")
    private double kIndexerVelocityGoal = 0.0;

    @AutoLogOutput(key = "Shooter/FlywheelVelocityGoalRPS")
    private double kFlywheelVelocityGoal = 0.0;

    // Flywheel feedforward and feedback declarations
    private final PIDController flywheelPIDController;
    private SimpleMotorFeedforward flywheelFeedforward;
    private SlewRateLimiter flyhweelAccelerationLimiter;

    // Indexer feedforward declaration
    private final PIDController indexerPIDController;
    private SimpleMotorFeedforward indexerFeedforward;
    private SlewRateLimiter indexerAccelerationLimiter;

    // Hooder feedforward and feedback declarations
    private final ProfiledPIDController hooderPIDController;
    private ArmFeedforward hooderFeedforward;
    private TrapezoidProfile hooderTrapezoidProfile;

    public ShooterIOSim(FlywheelGains flywheelGains, IndexerGains indexerGains, HooderGains hooderGains) {
        kFlywheel = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX44Foc(1), 0.001, 0.1),
            DCMotor.getKrakenX44Foc(1)
        );
        kIndexer = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44Foc(1), 0.001, 0.1),
            DCMotor.getKrakenX44Foc(1)
        );
        System.out.println(DCMotor.getKrakenX44Foc(1));
        kHooder = new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(DCMotor.getKrakenX44Foc(1), 0.001, 50.0), 
            DCMotor.getKrakenX44Foc(1), 
            50.0, 
            ShooterConstants.getInstance().kHooderArmLengthMeters,
            Units.degreesToRadians(0),
            Units.degreesToRadians(90),
            true,
            0.0
        );

        // Flywheel feedforward and feedback initialization
        flywheelPIDController = new PIDController(
            flywheelGains.p(), 
            flywheelGains.i(),
            flywheelGains.d()
        );

        flywheelFeedforward = new SimpleMotorFeedforward(flywheelGains.s(), flywheelGains.v(), flywheelGains.a());
        flyhweelAccelerationLimiter = new SlewRateLimiter(flywheelGains.maxAccelerationRadiansPerSecondSquared());

        // Indexer feedforward initialization
        indexerPIDController = new PIDController(
            indexerGains.p(), 
            indexerGains.i(),
            indexerGains.d()
        );

        indexerFeedforward = new SimpleMotorFeedforward(indexerGains.s(), indexerGains.v(), indexerGains.a());
        indexerAccelerationLimiter = new SlewRateLimiter(indexerGains.maxAccelerationRadiansPerSecondSquared());

        // Hooder feedforward and feedback initialization
        hooderTrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(hooderGains.maxVelocityRadiansPerSecond(), hooderGains.maxAccelerationRadiansPerSecondSquared()));

        hooderPIDController = new ProfiledPIDController(
            hooderGains.p(), 
            hooderGains.i(),
            hooderGains.d(),
            new TrapezoidProfile.Constraints(hooderGains.maxVelocityRadiansPerSecond(), hooderGains.maxAccelerationRadiansPerSecondSquared())
        );

        hooderFeedforward = new ArmFeedforward(hooderGains.s(), hooderGains.g(), hooderGains.v(), hooderGains.a());
    }

    @Override
    public void setFlywheelVoltage(double volts) {
        kFlywheel.setInputVoltage(volts);
    }

    @Override
    public void setFlywheelVelocity(double velocity) {
        kFlywheelVelocityGoal = velocity;
    }

    @Override
    public void stopFlywheel() {
        setFlywheelVelocity(ShooterConstants.getInstance().kFlywheelVelocityOffRadiansPerSec);
    }

    @Override
    public void setIndexerVoltage(double volts) {
        kIndexer.setInputVoltage(volts);
    }

    @Override
    public void setIndexerVelocity(double velocity) {
        kIndexerVelocityGoal = velocity;
    }

    @Override
    public void stopIndexer() {
        setIndexerVelocity(ShooterConstants.getInstance().kIndexerVelocityOffRadiansPerSec);
    }
    
    @Override
    public void setHooderVoltage(double volts) {
        kHooder.setInputVoltage(volts);
    }

    @Override
    public void setHooderPositionRotationsGoal(Rotation2d newHoodPosition) {
        kHooderPositionRadiansGoal = newHoodPosition.getRadians();
        hooderPIDController.setGoal(kHooderPositionRadiansGoal);
    }

    @Override
    public void stopHooder() {
        kHooder.setInputVoltage(0.0);
    }

    @Override
    public double getHooderPositionRadiansGoal() {
        return kHooderPositionRadiansGoal;
    }

    public double calculateFlywheelVolts(double goalVelocity) {
        double PIDVolts = flywheelPIDController.calculate(kFlywheel.getAngularVelocityRadPerSec(), goalVelocity);
        double feedforwardVolts = flywheelFeedforward.calculate(goalVelocity);

        return (PIDVolts + feedforwardVolts);
    }

    public double calculateIndexerVolts(double goalVelocity) {
        double PIDVolts = indexerPIDController.calculate(kIndexer.getAngularVelocityRadPerSec(), goalVelocity);
        double feedforwardVolts = indexerFeedforward.calculate(goalVelocity);

        return (PIDVolts + feedforwardVolts);
    }

    public double calculateHooderVolts(double goalPosition) {
        double PIDVolts = hooderPIDController.calculate(kHooder.getAngleRads(), goalPosition);
        double setpointVelocity = hooderPIDController.getSetpoint().velocity;
        double feedforwardVolts = hooderFeedforward.calculate(kHooder.getAngleRads(), setpointVelocity);

        return (PIDVolts + feedforwardVolts);
    }

    @Override
    public void updateInputs(ShooterInputs toUpdate) {
        kFlywheel.setInputVoltage(calculateFlywheelVolts(flyhweelAccelerationLimiter.calculate(kFlywheelVelocityGoal)));
        kIndexer.setInputVoltage(calculateIndexerVolts(indexerAccelerationLimiter.calculate(kIndexerVelocityGoal)));
        kHooder.setInputVoltage(calculateHooderVolts(kHooderPositionRadiansGoal));

        kFlywheel.update(kLoopPeriodSec);
        kIndexer.update(kLoopPeriodSec);
        kHooder.update(kLoopPeriodSec);

        // Flywheel
        toUpdate.flywheelOk = true;
        toUpdate.flywheelVoltage = (kFlywheel.getInputVoltage() > 0.0) ? 12.0 : 0.0;
        toUpdate.flywheelVelocityRPM = kFlywheel.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);
        toUpdate.flywheelStatorCurrent = (kFlywheel.getInputVoltage() > 0.0) ? kFlywheel.getCurrentDrawAmps() : 0.0;
        toUpdate.flywheelSupplyCurrent = (kFlywheel.getInputVoltage() > 0.0) ? kFlywheel.getCurrentDrawAmps() : 0.0;

        // Indexer
        toUpdate.indexerOk = true;
        toUpdate.indexerPositionRotations = kIndexer.getAngularPositionRotations();
        toUpdate.indexerVelocityRPM = kIndexer.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);
        toUpdate.indexerVoltage = kIndexer.getInputVoltage();
        toUpdate.indexerStatorCurrent = (kIndexer.getInputVoltage() > 0.0) ? kIndexer.getCurrentDrawAmps() : 0.0;
        toUpdate.indexerSupplyCurrent = (kIndexer.getInputVoltage() > 0.0) ? kIndexer.getCurrentDrawAmps() : 0.0;

        // Hooder
        toUpdate.hooderOk = true;
        toUpdate.hooderAngleRads = kHooder.getAngleRads();
        toUpdate.hooderVelocityRPM = kHooder.getVelocityRadPerSec();
        toUpdate.hooderVoltage = kHooder.getInput(0);
        toUpdate.hooderStatorCurrent = (kHooder.getInput(0) > 0.0) ? kHooder.getCurrentDrawAmps() : 0.0;
        toUpdate.hooderSupplyCurrent = (kHooder.getInput(0) > 0.0) ? kHooder.getCurrentDrawAmps() : 0.0;
    }

    @Override
    public void setHooderGains(double p, double i, double d, double s, double g, double v, double a)  {
        hooderPIDController.setPID(p, i, d);
        hooderFeedforward = new ArmFeedforward(s, g, v, a);
    }

    @Override
    public void setHooderMotionMagicConstraints(double maxVelocity, double maxAcceleration) {
        hooderTrapezoidProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    }
}
