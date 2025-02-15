package frc.team1126.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1126.Constants.CoralConstants;

public class CoralAcquisition extends SubsystemBase {
   
    private SparkFlex coralWheels;
    private SparkMax coralPivot;

    private SparkFlexConfig wheelConfig;
    private SparkMaxConfig pivotConfig;

    private AbsoluteEncoder pivotEncoder;

    private PIDController pivotController;

    public CoralAcquisition() {
        coralWheels = new SparkFlex(CoralConstants.CORAL_WHEELS_ID, MotorType.kBrushless);
        coralPivot = new SparkMax(CoralConstants.CORAL_PIVOT_ID, MotorType.kBrushless);

        wheelConfig = new SparkFlexConfig();
        pivotConfig = new SparkMaxConfig();

        pivotEncoder = coralPivot.getAbsoluteEncoder();

        pivotController = new PIDController(0, 0, 0);

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
        double angle = 90.0;
        if(getAngle() < angle) {
            double error =  getAngle() - angle;
            double output = pivotController.calculate(error);
            double feedforward = 0.1 * angle;

            coralPivot.set(output + feedforward);
        }
    }
    
    public void moveIn() {
        double angle = -60.0;
        if(getAngle() > angle) {
            double error =  getAngle() - angle;
            double output = pivotController.calculate(error);
            double feedforward = 0.1 * angle;

            coralPivot.set(-1 * (output + feedforward));
        }
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

    public void moveAcq(double speed) {
        coralPivot.set(speed);
    }

    public double getAngle() {
        return pivotEncoder.getPosition();
    }
}
