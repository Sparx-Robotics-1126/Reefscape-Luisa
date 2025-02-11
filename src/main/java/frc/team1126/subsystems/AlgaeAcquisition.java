package frc.team1126.subsystems;

// import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.team1126.Constants.AlgaeConstants;

public class AlgaeAcquisition extends SubsystemBase {
    private SparkMax algaeWheels;
    private SparkMax algaeRotation; 

    private SparkAbsoluteEncoder rotationEncoder;
    
    private SparkMaxConfig wheelConfig;
    private SparkMaxConfig rotationConfig;

    //private SparkClosedLoopController pidController;
    private PIDController pidController;


    //Creates a new AlgaeAcquisition.
    public AlgaeAcquisition() {
         
        algaeWheels = new SparkMax(AlgaeConstants.ALGAE_WHEELS_ID, MotorType.kBrushless);
         algaeRotation = new SparkMax(AlgaeConstants.ALGAE_ROTATION_ID, MotorType.kBrushless);

         rotationEncoder = algaeRotation.getAbsoluteEncoder();

         wheelConfig = new SparkMaxConfig();
         rotationConfig = new SparkMaxConfig();

        configurePID();
        configureSparkMaxes();
        }

    /*
     * configure PID settings here.
     */
    private void configurePID(){
        pidController = new PIDController(0, 0, 0);
    }

    /**
     * Add any extra SparkMax setting here.
     */
    private void configureSparkMaxes() {

        algaeWheels.configure(wheelConfig, null, null);
        algaeRotation.configure(rotationConfig, null, null);
        
    }

    //for testing
    public void spinAlgaeWheels() {
        algaeWheels.set(10);
    }

    //for testing
    public void moveArm(double speed) {
        algaeRotation.set(speed);
    }

    public void moveToAngle(double angle) {
       double targetAngle = angle;
        double currentAngle = getAngle();

        if(currentAngle < 0){
            currentAngle = 0;
        }
        
        if (currentAngle < 90) {
            double error =  currentAngle - targetAngle;
            double output = pidController.calculate(error);
            double feedforward = 0.1 * targetAngle;

            algaeRotation.set(output + feedforward);
        }
    }

    public double getAngle() {
        return rotationEncoder.getPosition();
    }
}
