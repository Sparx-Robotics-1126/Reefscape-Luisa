package frc.team1126.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1126.Constants.CoralConstants;

public class CoralAcquisition extends SubsystemBase {
   
    private SparkFlex coralWheels;
    private SparkMax coralPivot;

    private SparkFlexConfig wheelConfig;
    private SparkMaxConfig pivotConfig;

    public CoralAcquisition() {
        coralWheels = new SparkFlex(CoralConstants.CORAL_WHEELS_ID, MotorType.kBrushless);
        coralPivot = new SparkMax(CoralConstants.CORAL_PIVOT_ID, MotorType.kBrushless);

        wheelConfig = new SparkFlexConfig();
        pivotConfig = new SparkMaxConfig();

        configureSparkMaxes();
    }

    /**
     * Add any extra SparkMax setting here.
     */
    private void configureSparkMaxes() {
        coralWheels.configure(wheelConfig, null, null);
        coralPivot.configure(pivotConfig, null, null);
    }
   
    public void moveOut() {
        //method to move the entire coral acquisition out with the flex
    }
    
    public void moveIn() {
        //method to move the entire coral acquisition out with the flex
    }

    public void aquireCoral() {
        coralWheels.set(1);
    }

    public void releaseCoral() {
        coralWheels.set(-1);
    }

    public void toggleAcq() {
        PneumaticSubsystem.acqSolenoid.toggle();
    }
}
