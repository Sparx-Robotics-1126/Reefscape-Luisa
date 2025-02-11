package frc.team1126.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.team1126.commands.subsystems.pneumatics.FirePiston;

public class PneumaticSubsystem extends SubsystemBase {
    protected Compressor compressor;
    protected String dashboardKey;
    protected static Solenoid acqSolenoid;
    protected static Solenoid climbSolenoid;

    /**
     * Instantiates the input class with the potentiometer. For now this is all
     * the class does, so we're going to directly instantiate the AnalogInput.
     * 
     * @param potentiometerChannel the channel that the potentiometer is on for
     *                             the roboRio.
     */
    public PneumaticSubsystem() {
        compressor = new Compressor(20, PneumaticsModuleType.REVPH);
        acqSolenoid = new Solenoid(20, PneumaticsModuleType.REVPH, 0);
        climbSolenoid = new Solenoid(20, PneumaticsModuleType.REVPH, 1);
        dashboardKey = "Compressor Pressure";
    }

    /**
     * Gets the live voltage reading from the potentiometer as it is in the given
     * moment.
     * 
     * @return
     */
    public boolean isCompressorEnabledCommand() {
        return compressor.isEnabled();
    }

    public void enableCompressor() {
        compressor.enableDigital();
    }

    public void disableCompressor() {
        compressor.disable();
    }

    public boolean isSolenoidEnabled(Solenoid solenoid){
        return !solenoid.isDisabled();
    }

    public String getDashboardKey() {
        return dashboardKey;
    }

    public void setDashboardKey(String dashboardKey) { 
        this.dashboardKey = dashboardKey;
    }

    /**
     * This should update the dashboard with the voltage from the potentiometer.
     */
    public void updateDashboard() {
        SmartDashboard.putBoolean(dashboardKey, compressor.getPressureSwitchValue());
    }

    @Override
    public void periodic() {
        updateDashboard();
    }
}
