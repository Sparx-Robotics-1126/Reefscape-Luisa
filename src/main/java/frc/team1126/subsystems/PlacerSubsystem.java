package frc.team1126.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1126.Constants.PlacerConstants;

public class PlacerSubsystem extends SubsystemBase {

    private SparkMax placer;
    private SparkMax placerFollower;
    private SparkMaxConfig placerConfig;
    private SparkMaxConfig placerFollowerConfig;
    private DigitalInput bottomSensor;
    private DigitalInput topSensor;

    public PlacerSubsystem() { 
        placer = new SparkMax(PlacerConstants.PLACER_ID, MotorType.kBrushless);
        placerFollower = new SparkMax(PlacerConstants.PLACER_FOLLOWER_ID, MotorType.kBrushless);

        placerConfig = new SparkMaxConfig();
        placerFollowerConfig = new SparkMaxConfig();

        bottomSensor = new DigitalInput(PlacerConstants.PLACER_BOTTOM_ID);
        // topSensor = new DigitalInput(PlacerConstants.PLACER_TOP_ID);
        
        configureSparkMaxes();
    }

    /**
     * Add any extra SparkMax setting here.
     */
    private void configureSparkMaxes() {
        placerFollowerConfig.follow(PlacerConstants.PLACER_ID,true);
        placer.configure(placerConfig,  SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        placerFollower.configure(placerFollowerConfig,  SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    
    }

    /**
     * Moves the coral to the correct position to place
     */
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

    /**
     * Releases the coral
     */
    public void release() {
        placer.set(-1);
    }
    
    public void movePlacer(double speed){
        placer.set(speed);
    }

    /**
     * Returns if the top sensor sees the coral
     * @return true if the top sensor sees the coral, false otherwise
     */
    public boolean topHasCoral() {
        return topSensor.get();
    }


    /**
     * Returns if the bottom sensor sees the coral
     * @return true if the bottom sensor sees the coral, false otherwise
     */
    public boolean bottomHasCoral() {
        return !bottomSensor.get();
    }

    public double getSpeed(){
        return placer.get();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Bottom sensor", bottomHasCoral());
    }
}
