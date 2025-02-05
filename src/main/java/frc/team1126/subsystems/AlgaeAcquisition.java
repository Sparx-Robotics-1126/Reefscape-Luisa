package frc.team1126.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.team1126.Constants.AlgaeConstants;

public class AlgaeAcquisition {
    private SparkMax algaeWheels;
    private SparkMax algaeRotation; 
    
    private SparkMaxConfig wheelConfig;
    private SparkMaxConfig rotationConfig;


    public AlgaeAcquisition() {
         algaeWheels = new SparkMax(AlgaeConstants.ALGAE_WHEELS_ID, MotorType.kBrushless);
         algaeRotation = new SparkMax(AlgaeConstants.ALGAE_ROTATION_ID, MotorType.kBrushless);

         wheelConfig = new SparkMaxConfig();
         rotationConfig = new SparkMaxConfig();

         configureSparkMaxes();

        }

    /**
     * Add any extra SparkMax setting here.
     */
    private void configureSparkMaxes() {

        algaeWheels.configure(wheelConfig, null, null);
        algaeRotation.configure(rotationConfig, null, null);
        
    }


    public void spinAlgaeWheels() {

    }

    public void moveToPosition() {

    }

    public void brakeAlgae() {

    }
}
