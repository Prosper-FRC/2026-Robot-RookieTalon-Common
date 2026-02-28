package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ShooterIOSim implements ShooterIO {
    private static final double kLoopPeriodSec = 0.02;

    private final FlywheelSim kFlywheel;
    private final DCMotorSim kIndexer;
    private final SingleJointedArmSim kHooder;

    public ShooterIOSim() {
        kFlywheel = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX44Foc(1), 0.1, 0.1),
            DCMotor.getKrakenX44Foc(1)
        );
        kIndexer = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(ShooterConstants.kIndexerMaxVelocityRPM, ShooterConstants.kIndexerMaxAccelerationRPM),
             DCMotor.getKrakenX44Foc(1)
        );
        System.out.println(DCMotor.getKrakenX44Foc(1));
        kHooder = new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(DCMotor.getKrakenX44Foc(1), 0.1, 0.1), 
            DCMotor.getKrakenX44Foc(1), 0.1, 
            ShooterConstants.kHooderArmLengthMeters,
            0.0,
            0.1,
            true,
            0.0
        );
    }

    @Override
    public void setIndexerVoltage(double volts) {
        kIndexer.setInputVoltage(volts);
    }

    @Override
    public void setIndexerVelocity(double velocity) {
        kIndexer.setAngularVelocity(velocity);
    }

    @Override
    public void stopIndexer() {
        kIndexer.setInputVoltage(0.0);
    }

    @Override
    public void setFlywheelVoltage(double volts) {
        kFlywheel.setInputVoltage(volts);
    }

    @Override
    public void setFlywheelVelocity(double velocity) {
        kFlywheel.setAngularVelocity(velocity);
    }

    @Override
    public void stopFlywheel() {
        kFlywheel.setInputVoltage(0.0);
    }
    
    @Override
    public void setHooderVoltage(double volts) {
        kHooder.setInputVoltage(volts);
    }

    @Override
    public void setHooderPositionRotations(Rotation2d newHoodPosition) {
        kHooder.setState(newHoodPosition.getRadians(), kHooder.getVelocityRadPerSec());
    }

    @Override
    public void stopHooder() {
        kHooder.setInputVoltage(0.0);
    }

    @Override
    public void updateInputs(ShooterInputs toUpdate) {
        kFlywheel.setInputVoltage((kFlywheel.getInputVoltage() > 0.0) ? 12.0 : 0.0);
        kIndexer.setInputVoltage((kFlywheel.getInputVoltage() > 0.0) ? 12.0 : 0.0);
        kHooder.setInputVoltage((kFlywheel.getInputVoltage() > 0.0) ? 12.0 : 0.0);

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
        toUpdate.indexerVoltage = (kIndexer.getInputVoltage() > 0.0) ? 12.0 : 0.0;
        toUpdate.indexerStatorCurrent = (kIndexer.getInputVoltage() > 0.0) ? kIndexer.getCurrentDrawAmps() : 0.0;
        toUpdate.indexerSupplyCurrent = (kIndexer.getInputVoltage() > 0.0) ? kIndexer.getCurrentDrawAmps() : 0.0;

        // Hooder
        toUpdate.hooderOk = true;
        toUpdate.hooderAngleRads = kHooder.getAngleRads();
        toUpdate.hooderVelocityRPM = kHooder.getVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);
        toUpdate.hooderVoltage = (kHooder.getInput(0) > 0.0) ? 12.0 : 0.0;
        toUpdate.hooderStatorCurrent = (kHooder.getInput(0) > 0.0) ? kHooder.getCurrentDrawAmps() : 0.0;
        toUpdate.hooderSupplyCurrent = (kHooder.getInput(0) > 0.0) ? kHooder.getCurrentDrawAmps() : 0.0;
    }
}
