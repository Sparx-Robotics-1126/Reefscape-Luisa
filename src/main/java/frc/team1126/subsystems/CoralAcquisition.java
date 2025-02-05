package frc.team1126.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.team1126.Constants.CoralConstants;

public class CoralAcquisition {
   
    private SparkFlex coralWheels;
    private SparkMax coralPivot;

    private SparkFlexConfig wheelConfig;
    private SparkMaxConfig pivotConfig;

    public CoralAcquisition() {
        coralWheels = new SparkFlex(CoralConstants.CORAL_WHEELS_ID, MotorType.kBrushless);
        coralPivot = new SparkMax(CoralConstants.CORAL_PIVOT_ID, MotorType.kBrushless);

        wheelConfig = new SparkFlexConfig();
        pivotConfig = new SparkMaxConfig();

        //ADD PNEUMATICS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        configureSparkMaxes();
    }

    /**
     * Add any extra SparkMax setting here.
     */
    private void configureSparkMaxes() {
        coralWheels.configure(wheelConfig, null, null);
        coralPivot.configure(pivotConfig, null, null);
    }
   
    private void moveOut() {

    }
    
    private void moveIn() {

    }

    private void aquire() {

    }
}
