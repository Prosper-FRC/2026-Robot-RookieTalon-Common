package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ShooterIO {
    @AutoLog
    static class ShooterInputs {
        public boolean indexerOk = false;
        public double indexerPositionRotations = 0.0d;
        public double indexerVelocityRPM = 0.0d;
        public double indexerTemperatureCelcius = 0.0d;
        public double indexerVoltage = 0.0d;
        public double indexerStatorCurrent = 0.0d;
        public double indexerSupplyCurrent = 0.0d;

        public boolean flywheelOk = false;
        public double flywheelPositionRotations = 0.0d;
        public double flywheelVelocityRPM = 0.0d;
        public double flywheelTemperatureCelcius = 0.0d;
        public double flywheelVoltage = 0.0d;
        public double flywheelStatorCurrent = 0.0d;
        public double flywheelSupplyCurrent = 0.0d;

        public boolean hooderOk = false;
        public double hooderAngleRads = 0.0d;
        public double hooderVelocityRPM = 0.0d;
        public double hooderTemperatureCelcius = 0.0d;;
        public double hooderVoltage = 0.0d;
        public double hooderStatorCurrent = 0.0d;
        public double hooderSupplyCurrent = 0.0d; 
    }

    default public void updateInputs(ShooterInputs toUpdate) {} 

    default public void setIndexerVoltage(double volts) {}

    default public void setIndexerVelocity(double velocity) {}

    default public void stopIndexer() {}

    default public void resetIndexer() {}

    default public void setFlywheelVoltage(double volts) {}

    default public void setFlywheelVelocity(double velocity) {}

    default public void stopFlywheel() {}

    default public void resetFlywheel() {}
    
    default public void setHooderPositionRotations(Rotation2d newHoodPosition) {}

    default public void setHooderVoltage(double volts) {}

    default public void stopHooder() {}

    default public void resetHooder() {}
}