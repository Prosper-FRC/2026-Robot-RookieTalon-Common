package frc.robot.Subsystems.Shooter;

public class IndexerSim implements IndexerIO {
    
    private double m_voltage = 0.0;
    private boolean m_indexerRunning = false;
    @Override

    public void setIndexerVoltage(double volts) {
        m_voltage = volts;
        m_indexerRunning = volts != 0.0;

    }

     @Override
    public void stopIndexer() {
        m_voltage = 0.0;
        m_indexerRunning = false;
    }

    public double getVoltage() {
        return m_voltage;
    }

    public boolean isRunning() {
        return m_indexerRunning;
    }

}


