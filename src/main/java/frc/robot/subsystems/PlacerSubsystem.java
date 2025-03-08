package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PlacerConstants;

public class PlacerSubsystem extends SubsystemBase {

    private SparkMax placer;
    private SparkMax placerFollower;
    private SparkMaxConfig placerConfig;
    private SparkMaxConfig placerFollowerConfig;
    private DigitalInput bottomSensor;

    public PlacerSubsystem() { 
        placer = new SparkMax(PlacerConstants.PLACER_ID, MotorType.kBrushless);
        placerFollower = new SparkMax(PlacerConstants.PLACER_FOLLOWER_ID, MotorType.kBrushless);

        placerConfig = new SparkMaxConfig();
        placerFollowerConfig = new SparkMaxConfig();

        bottomSensor = new DigitalInput(PlacerConstants.PLACER_BOTTOM_ID);
        
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
     * Releases the coral
     */
    public void release() {
        placer.set(-1);
    }
    
    public void movePlacer(double speed){
        placer.set(speed);
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
