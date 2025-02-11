package frc.team1126.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PlacerSubsystem extends SubsystemBase {

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
    public void grab() {
        placer.set(1);
    }

    public void release() {
        placer.set(-1);
    }
}
