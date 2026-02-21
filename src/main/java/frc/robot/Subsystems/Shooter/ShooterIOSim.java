package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
    private static final double kLoopPeriodSec = 0.02;
    
    private double indexerVoltage = 0.0;
    private double indexerVelocity = 0.0;
    private boolean indexerRunning = false;

    private double hooderVoltage = 0.0;
    private Rotation2d hooderPositionRotations = new Rotation2d(0);

    private final FlywheelSim kFlywheel;

    public ShooterIOSim() {
        kFlywheel = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX44Foc(1), 0.1, 0.1),
            DCMotor.getKrakenX44Foc(1)
        );
    }

    @Override
    public void setIndexerVoltage(double volts) {
        indexerVoltage = volts;
    }

    @Override
    public void setIndexerVelocity(double velocity) {
        indexerVelocity = velocity;
    }

    @Override
    public void stopIndexer() {
        indexerVoltage = 0.0;
        indexerRunning = false;
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
        hooderVoltage = volts;
    }

    @Override
    public void setHooderPositionRotations(Rotation2d newHoodPosition) {
        hooderPositionRotations = newHoodPosition;
    }

    @Override
    public void stopHooder() {
        hooderVoltage = 0.0;
    }

    @Override
    public void updateInputs(ShooterInputs toUpdate) {
        kFlywheel.setInputVoltage((kFlywheel.getInputVoltage() > 0.0) ? 12.0 : 0.0);

        kFlywheel.update(kLoopPeriodSec);

        // Flywheel
        toUpdate.flywheelOk = true;
        toUpdate.flywheelVoltage = (kFlywheel.getInputVoltage() > 0.0) ? 12.0 : 0.0;
        toUpdate.flywheelVelocityRPM = kFlywheel.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);
        toUpdate.flywheelStatorCurrent = (kFlywheel.getInputVoltage() > 0.0) ? kFlywheel.getCurrentDrawAmps() : 0.0;
        toUpdate.flywheelSupplyCurrent = (kFlywheel.getInputVoltage() > 0.0) ? kFlywheel.getCurrentDrawAmps() : 0.0;

        // Indexer
        toUpdate.indexerOk = true;

        // Hooder
        toUpdate.hooderOk = true;
    }
}
