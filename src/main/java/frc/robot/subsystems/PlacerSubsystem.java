package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfigurator;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.PlacerConstants;


public class PlacerSubsystem extends SubsystemBase {

    private SparkMax placer;
    private SparkMax placerFollower;
    private SparkMaxConfig placerConfig;
    private SparkMaxConfig placerFollowerConfig;
    private CANrange bottomSensor;
    private CANrangeConfigurator bottomSensorConfig;
    private Trigger hasGamepiece;


    public PlacerSubsystem() { 
        placer = new SparkMax(PlacerConstants.PLACER_ID, MotorType.kBrushless);
        placerFollower = new SparkMax(PlacerConstants.PLACER_FOLLOWER_ID, MotorType.kBrushless);

        placerConfig = new SparkMaxConfig();
        placerFollowerConfig = new SparkMaxConfig();

        bottomSensor = new CANrange(36);
        bottomSensorConfig = bottomSensor.getConfigurator();
       hasGamepiece =  new Trigger(this::bottomHasCoral).debounce(0);
        
        configureSparkMaxes();
    }

    /**
     * Add any extra SparkMax setting here.
     */
    private void configureSparkMaxes() {
        CANrangeConfiguration config = new CANrangeConfiguration();
        ProximityParamsConfigs proxConfig = new ProximityParamsConfigs();
        config.ProximityParams.ProximityThreshold = 0.1;
        proxConfig.MinSignalStrengthForValidMeasurement = 9000;
        placerFollowerConfig.follow(PlacerConstants.PLACER_ID,true);
        placer.configure(placerConfig,  SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        placerFollower.configure(placerFollowerConfig,  SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        bottomSensorConfig.apply(config);
        bottomSensorConfig.apply(proxConfig);
        

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

        return bottomSensor.getIsDetected().getValue();
    }

    public double getSpeed(){
        return placer.get();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Bottom sensor", bottomHasCoral());
        SmartDashboard.putNumber("Placer distance", bottomSensor.getDistance().getValueAsDouble());
    }
}
