package frc.team1126.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.team1126.Constants.ArmConstants;

public class ArmSubsystem {
    private SparkMax turn1;
    private SparkMax turn2; // follows turn1
    private SparkMax elevator;

    private SparkMaxConfig turn1Config;
    private SparkMaxConfig turn2Config;
    private SparkMaxConfig elevatorConfig;


    public ArmSubsystem() {
        turn1 = new SparkMax(ArmConstants.TURN_ONE_ID, MotorType.kBrushless);
        turn2 = new SparkMax(ArmConstants.TURN_TWO_ID, MotorType.kBrushless);
        elevator = new SparkMax(ArmConstants.ELEVATOR_ID, MotorType.kBrushless);

        turn1Config = new SparkMaxConfig();
        turn2Config = new SparkMaxConfig();
        elevatorConfig = new SparkMaxConfig();

        configureSparkMaxes();

    }

    /**
     * Add any extra SparkMax setting here.
     */
    private void configureSparkMaxes() {
       
        turn2Config.follow(ArmConstants.TURN_ONE_ID);

        turn2.configure(turn2Config, null, null);
        turn1.configure(turn1Config, null, null);
        elevator.configure(elevatorConfig, null, null);
    
    }


    public void moveToExtension() {

    }

    public void extensionToHome() {

    }
   
    public void angleToHome() {

    }
   
    public void armBrake() {

    }

    public void moveToAngle() {

    }



}
