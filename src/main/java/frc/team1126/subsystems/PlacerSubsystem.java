package frc.team1126.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1126.Constants.PlacerConstants;

public class PlacerSubsystem extends SubsystemBase {

    private SparkMax placer;
    private SparkMaxConfig placerConfig;

    private DigitalInput bottomSensor;
    private DigitalInput topSensor;

    public PlacerSubsystem() { 
        placer = new SparkMax(PlacerConstants.PLACER_ID, null);
        placerConfig = new SparkMaxConfig();

        bottomSensor = new DigitalInput(PlacerConstants.PLACER_BOTTOM_ID);
        topSensor = new DigitalInput(PlacerConstants.PLACER_TOP_ID);
        
        configureSparkMaxes();
    }

    /**
     * Add any extra SparkMax setting here.
     */
    private void configureSparkMaxes() {
        placer.configure(placerConfig, null, null);
    }

    public void grab() {
        if (!bottomHasCoral()) { // if the coral isnt in the placer yet
            placer.set(1);
        } else if (!topHasCoral() && bottomHasCoral()) { // if the coral is sensed by the bottom, but yet to get to the top
            placer.set(1);
        } else if (topHasCoral() && !bottomHasCoral()) { // if the coral is sensed by the top, but overshoots bottom
            placer.set(-1);
        } else { // if the coral is sensed by both top and bottom
            placer.set(0);
        }
    }

    public void release() {
        placer.set(-1);
    }

    public boolean topHasCoral() {
        return topSensor.get();
    }

    public boolean bottomHasCoral() {
        return bottomSensor.get();
    }
}
