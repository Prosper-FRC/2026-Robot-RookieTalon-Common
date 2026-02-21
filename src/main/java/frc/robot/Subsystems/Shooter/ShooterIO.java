package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    static class ShooterInputs {
        public boolean isOk = false;
        public double shooterPositionRotations = 0.0d;
        public double shooterVelocityRPS = 0.0d;
        public double shooterTemperatureCelcius = 0.0d;
        public double shooterVoltage = 0.0d;
        public double shooterStatorCurrent = 0.0d;
        public double shooterSupplyCurrent = 0.0d; 
    }

    default public void updateInputs(ShooterInputs toUpdate) {} 

    default public void setShooterVoltage() {}

    default public void setShooterVelocityRPM() {}

    default public void stopShooter() {}

    default public void resetShooter() {}
}
