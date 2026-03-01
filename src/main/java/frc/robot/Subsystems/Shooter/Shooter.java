package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final ShooterIO kShooter;
    private final ShooterInputsAutoLogged kShooterInputs;

    public enum HooderPosition {
        kHoodPosition1(ShooterConstants.getInstance().kHoodPosition1),
        kHoodPosition2(ShooterConstants.getInstance().kHoodPosition2),
        kHoodPosition3(ShooterConstants.getInstance().kHoodPosition3);

        public final Rotation2d angle;

        HooderPosition(Rotation2d angle) {
            this.angle = angle;
        }
    }

    @AutoLogOutput(key = "Shooter/ShooterOn")
    private boolean shooterOn = false;

    @AutoLogOutput(key = "Shooter/FlywheelReady")
    private boolean flywheelReady = false;

    @AutoLogOutput(key = "Shooter/FlywheelError")
    private double flywheelError = 0.0;

    @AutoLogOutput(key = "Shooter/FlywheelGoalRPM")
    private double flywheelGoalRPM = 0.0;

    private HooderPosition currentHooderPosition = HooderPosition.kHoodPosition1;

    public Shooter(ShooterIO shooterIO) {
        kShooter = shooterIO;
        kShooterInputs = new ShooterInputsAutoLogged();
    }

    @Override
    public void periodic() {
        kShooter.updateInputs(kShooterInputs);

        Logger.processInputs("Shooter", kShooterInputs);

        Logger.recordOutput("Shooter/HooderPosition", currentHooderPosition);
    }

    // Flywheel IO Functions

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

    // Hood IO Functions

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

    public double getHooderPositionRadiansGoal() {
        return kShooter.getHooderPositionRadiansGoal();
    }

    // Hood
    public void nextPosition() {
        int newIndex = currentHooderPosition.ordinal() + 1;
        HooderPosition[] values = HooderPosition.values();
        
        if (newIndex < values.length) {setHoodPosition(values[newIndex]);}
    }

    public void previousPosition() {
        int newIndex = currentHooderPosition.ordinal() - 1;
        HooderPosition[] values = HooderPosition.values();
        
        if (newIndex >= 0) {setHoodPosition(values[newIndex]);}
    }

    public void setHoodPosition(HooderPosition newPos) {
        currentHooderPosition = newPos;
        setHooderPositionRotationsGoal(newPos.angle);
    }

    // Indexer IO Functions
    
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

    // Flywheel and Indexer Functions

    public void shooterOnOff() {
        if (shooterOn) {
            kShooter.stopFlywheel();
            kShooter.stopIndexer();
            flywheelGoalRPM = (ShooterConstants.getInstance().kFlywheelVelocityOffRadiansPerSec * 60) / (2 * Math.PI);
            shooterOn = false;
            flywheelReady = false;
        } else {
            if (flywheelReady) {
                kShooter.setIndexerVelocity(ShooterConstants.getInstance().kIndexerVelocityOnRadiansPerSec);
            } else {
                kShooter.setFlywheelVelocity(ShooterConstants.getInstance().kFlywheelVelocityOnRadiansPerSec);
            }
            shooterOn = true;
        }
    }

    public void checkFlywheelReady() {
        flywheelGoalRPM = (ShooterConstants.getInstance().kFlywheelVelocityOnRadiansPerSec * 60) / (2 * Math.PI);
        flywheelError = Math.abs(kShooterInputs.flywheelVelocityRPM - flywheelGoalRPM);
        if (flywheelError < ShooterConstants.getInstance().kFlywheelVelocityTolerance) {
            flywheelReady = true;
            kShooter.setIndexerVelocity(ShooterConstants.getInstance().kIndexerVelocityOnRadiansPerSec);
        }
    }
}
