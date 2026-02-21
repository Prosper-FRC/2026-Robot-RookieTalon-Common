package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface HooderIO {
    @AutoLog
    static class HooderInputs {
        public boolean isOk = false;
        public double hooderPositionRotations = 0.0d;
        public double hooderVelocityRPS = 0.0d;
        public double hooderTemperatureCelcius = 0.0d;;
        public double hooderVoltage = 0.0d;
        public double hooderStatorCurrent = 0.0d;
        public double hooderSupplyCurrent = 0.0d; 
    }

    default public void updateInputs(HooderInputs toUpdate) {} 

    default public void setHooderPositionRotations(Rotation2d newHoodPosition) {}

    default public void setHooderVoltage(double volts) {}

    default public void stopHooder() {}

    default public void resetHooder() {}
}
