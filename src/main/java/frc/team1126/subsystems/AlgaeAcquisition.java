package frc.team1126.subsystems;

// import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.team1126.Constants.AlgaeConstants;

public class AlgaeAcquisition extends SubsystemBase {
    private SparkMax algaeWheels;
    private SparkMax algaeRotation; 

    private SparkAbsoluteEncoder rotationEncoder;
    
    private SparkMaxConfig wheelConfig;
    private SparkMaxConfig rotationConfig;

    private SparkClosedLoopController rotationController;

    private DigitalInput homeSensor;
    private DigitalInput algaeSensor;

    private double kRotationkS= 0.0;
    private double kRotationkG = .762;
    private double kRotationkV =.762 ;
    private double kRotationkA = 0.0;

    protected ShuffleboardTab algaeTab;

    private double angle;

    ElevatorFeedforward m_feedforward =
            new ElevatorFeedforward(
                    kRotationkS,
                    kRotationkG,
                    kRotationkV,
                    kRotationkA);


    //Creates a new AlgaeAcquisition.
    public AlgaeAcquisition() {
        if (!RobotBase.isSimulation()){

         algaeWheels = new SparkMax(AlgaeConstants.ALGAE_WHEELS_ID, MotorType.kBrushless);
         algaeRotation = new SparkMax(AlgaeConstants.ALGAE_ROTATION_ID, MotorType.kBrushless);

         homeSensor = new DigitalInput(AlgaeConstants.ALGAE_HOME_ID);
         algaeSensor = new DigitalInput(AlgaeConstants.ALGAE_SENSOR_ID);

         rotationEncoder = algaeRotation.getAbsoluteEncoder();

         rotationController = algaeRotation.getClosedLoopController();

         algaeTab = Shuffleboard.getTab("AlgaeTab");

         wheelConfig = new SparkMaxConfig();
         rotationConfig = new SparkMaxConfig();

        configurePID();
        configureSparkMaxes();
        }

        }

    /**
     * configure PID settings here.
     */
    private void configurePID(){

        rotationConfig.closedLoop
                .p(0)
                .i(0)
                .d(0)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

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

    
    public void reachGoal(double goalDistance){
        rotationController.setReference(goalDistance,ControlType.kPosition, ClosedLoopSlot.kSlot0, m_feedforward.calculate(goalDistance));
    }

    public Command setGoal(double distance){
        return run(() -> reachGoal(distance));
    }

    /**
     * Returns the position of algae acq
     */
    public double getAngle() {
        return rotationEncoder.getPosition();
    }

    public boolean isAlgaeHome(){
        return !homeSensor.get();
    }

    public boolean hasAlgae(){
        return algaeSensor.get();
    }


    @Override
    public void periodic() {

        SmartDashboard.putBoolean("Has Algae", hasAlgae());
        SmartDashboard.putNumber("Algae Position",rotationEncoder.getPosition() );
    }
}
