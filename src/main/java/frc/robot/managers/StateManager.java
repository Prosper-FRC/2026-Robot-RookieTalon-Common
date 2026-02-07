package frc.robot.managers;

import frc.robot.managers.ManagerConstants.ClimbStates;
import frc.robot.managers.ManagerConstants.IndexerStates;
import frc.robot.managers.ManagerConstants.IntakeStates;
import frc.robot.managers.ManagerConstants.LauncherStates;
import frc.robot.managers.ManagerConstants.SwerveStates;
import frc.robot.managers.ManagerConstants.YoshiStates;

public class StateManager {
    private static StateManager instance;

    private SwerveStates swerveState;
    private ClimbStates climbState;
    private IntakeStates intakeState;
    private LauncherStates launcherState;
    private IndexerStates indexerState;
    private YoshiStates yoshiState;

    private StateManager() {
        swerveState = SwerveStates.IDLE;
        climbState = ClimbStates.IDLE;
        intakeState = IntakeStates.IDLE;
        launcherState = LauncherStates.IDLE;
        indexerState = IndexerStates.IDLE;
        yoshiState = YoshiStates.IDLE;
    }

    public static synchronized StateManager getInstance() {
        if (instance == null) {
            instance = new StateManager();
        }
        return instance;
    }

    // Getter Methods
    public SwerveStates getSwerveState() {
        return swerveState;
    }

    public ClimbStates getClimbState() {
        return climbState;
    }

    public IntakeStates getIntakeState() {
        return intakeState;
    }

    public LauncherStates getLauncherState() {
        return launcherState;
    }

    public IndexerStates getIndexerState() {
        return indexerState;
    }

    public YoshiStates getYoshiState() {
        return yoshiState;
    }

    // Setter Methods
    public void setSwerveState(SwerveStates state) {
        swerveState = state;
    }

    public void setClimbState(ClimbStates state) {
        climbState = state;
    }

    public void setIntakeState(IntakeStates state) {
        intakeState = state;
    }

    public void setLauncherState(LauncherStates state) {
        launcherState = state;
    }

    public void setIndexerState(IndexerStates state) {
        indexerState = state;
    }

    public void setYoshiState(YoshiStates state) {
        yoshiState = state;
    }
}
