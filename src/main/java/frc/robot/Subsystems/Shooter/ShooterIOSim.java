package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
    private double indexerVoltage = 0.0;
    private double indexerVelocity = 0.0;
    private boolean indexerRunning = false;

    private double flywheelVoltage = 0.0;
    private boolean flywheelRunning = false;

    private double hooderVoltage = 0.0;
    private Rotation2d hooderPositionRotations = new Rotation2d(0);

    private final FlywheelSim kFlywheel;

    public ShooterIOSim() {
        kFlywheel = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX44Foc(1), 0.1, 0.1),
            DCMotor.getKrakenX44Foc(1)
        );
    }

    public void setIndexerVoltage(double volts) {
        indexerVoltage = volts;
    }

    public void setIndexerVelocity(double velocity) {
        indexerVelocity = velocity;
    }

    public void stopIndexer() {
        indexerVoltage = 0.0;
        indexerRunning = false;
    }

    public void resetIndexer() {}

    public void setFlywheelVoltage(double volts) {
        flywheelVoltage = volts;
        if (flywheelVoltage > 0.0) {
            flywheelRunning = true;
        }

        kFlywheel.setInputVoltage(volts);
    }

    public void stopFlywheel() {
        flywheelVoltage = 0.0;
        flywheelRunning = false;
        kFlywheel.setInputVoltage(0.0);
    }

    public void resetFlywheel() {}
    
    public void setHooderVoltage(double volts) {
        hooderVoltage = volts;
    }

    public void setHooderPositionRotations(Rotation2d newHoodPosition) {
        hooderPositionRotations = newHoodPosition;
    }

    public void stopHooder() {
        hooderVoltage = 0.0;
    }

    public void resetHooder() {}

    public void updateInputs(ShooterInputs toUpdate) {

    }
}
