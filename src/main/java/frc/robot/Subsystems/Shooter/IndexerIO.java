package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    static class IndexerInputs {
        public boolean isOk = false;
        public double indexerPositionRotations = 0.0d;
        public double indexerVelocityRPS = 0.0d;
        public double indexerTemperatureCelcius = 0.0d;
        public double indexerVoltage = 0.0d;
        public double indexerStatorCurrent = 0.0d;
        public double indexerSupplyCurrent = 0.0d; 
    }

    default public void updateInputs(IndexerInputs toUpdate) {} 

    default public void setIndexerVoltage(double volts) {}

    default public void setIndexerVelocity(double velocity) {}

    default public void stopIndexer() {}

    default public void resetIndexer() {}
}
