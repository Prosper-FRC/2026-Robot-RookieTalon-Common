package frc.robot.Subsystems.Shooter;

public class Shooter {
    private final ShooterIO kShooter;
    private final ShooterInputsAutoLogged kShooterInputs = new ShooterInputsAutoLogged();

    private final HooderIO kHooder;
    private final HooderInputsAutoLogged kHooderInputs = new HooderInputsAutoLogged();
    
    public Shooter(ShooterIO shooterIO, HooderIO hooderIO) {
        kShooter = shooterIO;
        kHooder = hooderIO;
    }

    public void periodic() {
        kShooter.updateInputs(kShooterInputs);
        kHooder.updateInputs(kHooderInputs);
    }
}
