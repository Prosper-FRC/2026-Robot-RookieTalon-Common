package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Shooter.ShooterIO.ShooterInputs;

public class Shooter extends SubsystemBase {
    private final ShooterIO kShooter;
    private final ShooterInputs kShooterInputs;

    public Shooter(ShooterIO shooterIO) {
        kShooter = shooterIO;
        kShooterInputs = new ShooterInputs();
    }

    @Override
    public void periodic() {
        kShooter.updateInputs(kShooterInputs);

        Logger.processInputs("Shooter", (LoggableInputs) kShooterInputs);
    }

    /* FLYWHEEL */
    public void setFlywheelVoltage(double volts) {
        kShooter.setFlywheelVoltage(volts);
    }

    public void setFlywheelVelocity(double velocity) {
        kShooter.setFlywheelVelocity(velocity);
    }

    public void stopFlywheel() {
        kShooter.stopFlywheel();
    }

    public void resetFlywheel() {
        kShooter.resetFlywheel();
    }

    /* HOOD */

    public void setHooderVoltage(double volts) {
        kShooter.setHooderVoltage(volts);
    }
    
    public void setHooderPositionRotations(Rotation2d newHoodPosition) {
        kShooter.setHooderPositionRotations(newHoodPosition);
    }

    public void stopHooder() {
        kShooter.stopHooder();
    }
    
    public void resetHooder() {
        kShooter.resetHooder();
    }

    /* INDEXER */
    
    public void setIndexerVoltage(double volts) {
        kShooter.setIndexerVoltage(volts);
    }

    public void setIndexerVelocity(double velocity) {
        kShooter.setIndexerVelocity(velocity);
    }

    public void stopIndexer() {
        kShooter.stopIndexer();
    }

    public void resetIndexer() {
        kShooter.resetIndexer();
    }
}
