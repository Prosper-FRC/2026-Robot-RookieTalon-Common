package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final ShooterIO kShooter;
    private final ShooterInputsAutoLogged kShooterInputs;

    public Shooter(ShooterIO shooterIO) {
        kShooter = shooterIO;
        kShooterInputs = new ShooterInputsAutoLogged();
    }

    @Override
    public void periodic() {
        kShooter.updateInputs(kShooterInputs);

        Logger.processInputs("Shooter", kShooterInputs);
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
    
    public void setHooderPositionRotationsGoal(Rotation2d newHoodPosition) {
        kShooter.setHooderPositionRotationsGoal(newHoodPosition);
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
