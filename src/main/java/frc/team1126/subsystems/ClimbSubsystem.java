package frc.team1126.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1126.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
    private SparkMax climb;

    private SparkAbsoluteEncoder climbEncoder;

    private SparkMaxConfig climbConfig;

    private PIDController pidController;


    public ClimbSubsystem() {

        climb = new SparkMax(ClimbConstants.CLIMB_ID, MotorType.kBrushless);
        climbEncoder = climb.getAbsoluteEncoder();
        climbConfig = new SparkMaxConfig();

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
        climb.configure(climbConfig, null, null);
    }

    /*
     * 
     */
    public void moveClimbToPos(double angle) {
        double targetAngle = angle;
        double currentAngle = getAngle();

        if (currentAngle > -160 || currentAngle < 90 ) { // these should be made constants at some point!!
            double error =  currentAngle - targetAngle;
            double output = pidController.calculate(error);
            double feedforward = 0.1 * targetAngle;

            climb.set(output + feedforward);
        }
    }

    public void moveClimb(double speed){
        climb.set(speed);
    }

    public void toggleClimb() {
        PneumaticSubsystem.climbSolenoid.toggle();
    }

    public double getAngle() {
        return climbEncoder.getPosition();
    }
}
