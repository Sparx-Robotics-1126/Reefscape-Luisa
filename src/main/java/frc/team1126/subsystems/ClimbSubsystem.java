package frc.team1126.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.team1126.Constants.ClimbConstants;

public class ClimbSubsystem {
    private SparkMax climb;

    private SparkMaxConfig climbConfig;

    public ClimbSubsystem() {

        climb = new SparkMax(ClimbConstants.CLIMB_ID, MotorType.kBrushless);
        climbConfig = new SparkMaxConfig();

        //ADD PNEUMATICS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        configureSparkMaxes();
    }

    /**
     * Add any extra SparkMax setting here.
     */
    private void configureSparkMaxes() {
        climb.configure(climbConfig, null, null);
    }

    public void climb() {

    }

    public void extendClimb() {

    }

    public void brake() {

    }
}
