package frc.team1126.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class PlacerSubsystem {

    private SparkMax placer;
    private SparkMaxConfig placerConfig;

    public PlacerSubsystem() { 
        placer = new SparkMax(0, null);
        placerConfig = new SparkMaxConfig();
        
        configureSparkMaxes();
    }

    /**
     * Add any extra SparkMax setting here.
     */
    private void configureSparkMaxes() {
        placer.configure(placerConfig, null, null);
    }

    private void moveIn() {

    }

    private void moveOut() {

    }

    private void grab() {

    }

    private void release() {

    }
}
