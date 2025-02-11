package frc.team1126.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1126.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private SparkMax turnMotor;
    private SparkMax turnFollower; // follows turn1
    private SparkMax extension;

    private SparkAbsoluteEncoder turnEncoder;
    private SparkAbsoluteEncoder extensionEncoder;

    private PIDController pidController;

    private SparkMaxConfig turn1Config;
    private SparkMaxConfig turn2Config;
    private SparkMaxConfig extensionConfig;


    public ArmSubsystem() {
        turnMotor = new SparkMax(ArmConstants.TURN_ONE_ID, MotorType.kBrushless);
        turnFollower = new SparkMax(ArmConstants.TURN_TWO_ID, MotorType.kBrushless);
        extension = new SparkMax(ArmConstants.ELEVATOR_ID, MotorType.kBrushless);

        turnEncoder = turnMotor.getAbsoluteEncoder();
        extensionEncoder = extension.getAbsoluteEncoder();

        turn1Config = new SparkMaxConfig();
        turn2Config = new SparkMaxConfig();
        extensionConfig = new SparkMaxConfig();

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
       
        turn2Config.follow(ArmConstants.TURN_ONE_ID);

        turnFollower.configure(turn2Config, null, null);
        turnMotor.configure(turn1Config, null, null);
        extension.configure(extensionConfig, null, null);
    
    }

    public void moveToExtension(double position) {
        double targetPos = position;
        double currentPos = getExtension();

        if(currentPos < 0){
            currentPos = 0;
        }
    
        if (currentPos < 90) {
            double error =  currentPos - targetPos;
            double output = pidController.calculate(error);
            double feedforward = 0.1 * targetPos;

            extension.set(output + feedforward);
        }
    }

    public void extensionToHome() {
        while(extensionEncoder.getPosition() > 0) {
            extension.set(-0.5);
        }
    }
   
    public void angleToHome() {
        while(turnEncoder.getPosition() > 0) {
            turnMotor.set(-0.5);
        }
    }

    public void moveToAngle(double angle) {
        double targetAngle = angle;
        double currentAngle = getArmAngle();

        if(currentAngle < 0){
            currentAngle = 0;
        }
    
        if (currentAngle < 90) {
            double error =  currentAngle - targetAngle;
            double output = pidController.calculate(error);
            double feedforward = 0.1 * targetAngle;

            turnMotor.set(output + feedforward);
        }
    }

    public double getArmAngle() {
        return turnEncoder.getPosition();
    }

    public double getExtension() {
        return extensionEncoder.getPosition();
    }
}
