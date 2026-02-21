package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final IndexerIO kIndexer;
    private final IndexerInputsAutoLogged kIndexerInputs = new IndexerInputsAutoLogged();

    private final FlywheelIO kFlywheel;
    private final FlywheelInputsAutoLogged kFlywheelInputs = new FlywheelInputsAutoLogged();

    private final HooderIO kHooder;
    private final HooderInputsAutoLogged kHooderInputs = new HooderInputsAutoLogged();
    
    public Shooter(IndexerIO indexerIO, FlywheelIO flywheelIO, HooderIO hooderIO) {
        kIndexer = indexerIO;
        kFlywheel = flywheelIO;
        kHooder = hooderIO;
    }

    @Override
    public void periodic() {
        kIndexer.updateInputs(kIndexerInputs);
        kFlywheel.updateInputs(kFlywheelInputs);
        kHooder.updateInputs(kHooderInputs);

        Logger.processInputs("Indexer", kIndexerInputs);
        Logger.processInputs("Flywheel", kFlywheelInputs);
        Logger.processInputs("Hooder", kHooderInputs);
    }

    /* FLYWHEEL */
    
    public void setFlywheelVoltageFlywheel(double volts) {
        kFlywheel.setFlywheelVoltage(volts);
    }

    public void setFlywheelVelocity(double velocity) {
        kFlywheel.setFlywheelVelocity(velocity);
    }
    
    public void stopFlywheel() {
        kFlywheel.stopFlywheel();
    }

     public void resetFlywheel() {
        kFlywheel.resetFlywheel();
    }

    /* HOOD */

    public void setHooderVoltage(double volts) {
        kHooder.setHooderVoltage(volts);
    }
    
    public void setHooderPositionRotations(Rotation2d newHoodPosition) {
        kHooder.setHooderPositionRotations(newHoodPosition);
    }

    public void stopHooder() {
        kHooder.stopHooder();
    }
    
    public void resetHooder() {
        kHooder.resetHooder();
    }

    /* INDEXER */
    
    public void setIndexerVoltage(double volts) {
        kIndexer.setIndexerVoltage(volts);
    }

    public void sIndexertopIndexer() {
        kIndexer.stopIndexer();
    }

    public void sIndexeretIndexerVelocity(double velocity) {
        kIndexer.setIndexerVelocity(velocity);
    }

     public void resetIndexer() {
        kIndexer.resetIndexer();
    }
}
