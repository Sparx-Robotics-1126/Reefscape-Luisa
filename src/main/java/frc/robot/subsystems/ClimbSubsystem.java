package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
    private SparkMax climb;

    private RelativeEncoder climbEncoder;

    private SparkMaxConfig climbConfig;

    private SparkClosedLoopController pidController;

    private GenericEntry kClimbPEntry;
    private GenericEntry kClimbIEntry;
    private GenericEntry kClimbDEntry;

    protected ShuffleboardTab climbTab;

    private double angle;


    public ClimbSubsystem() {
  if (!RobotBase.isSimulation()){

        climb = new SparkMax(ClimbConstants.CLIMB_ID, MotorType.kBrushless);
        climbEncoder = climb.getEncoder();
        climbConfig = new SparkMaxConfig();

        pidController = climb.getClosedLoopController();
        climbTab = Shuffleboard.getTab("ClimbTab");
        initShuffleboard();
        configurePID();
        configureSparkMaxes();
  }
    }

    /*
     * configure PID settings here.
     */
    private void configurePID(){
//        pidController = new PIDController(0, 0, 0);
        double kClimbP = kClimbPEntry.getDouble(0);
        double kClimbI = kClimbIEntry.getDouble(0);
        double kClimbD = kClimbDEntry.getDouble(0);

        climbConfig.closedLoop
                .p(5)
                .i(kClimbI)
                .d(kClimbD)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);        
    }

    /**
     * Add any extra SparkMax setting here.
     */
    private void configureSparkMaxes() {
        climb.configure(climbConfig, null, null);
    }

    private void initShuffleboard(){
        kClimbPEntry = climbTab.add("Ext P", 0).getEntry();
        kClimbIEntry = climbTab.add("Ext I", 0).getEntry();
        kClimbDEntry = climbTab.add("Ext D", 0).getEntry();

        angle = climbTab.add("Angle", 0).getEntry().getDouble(0);
        climbTab.add("Current Climb Position",0);
    }
    
    /**
     * Moves the climber at a specific speed
     * @param speed the speed to move the climber
     */
    public void moveClimb(double speed){
        climb.set(speed);
    }
    
    /**
     * Returns the current position of the climber
     */
    public double getAngle() {
        return climbEncoder.getPosition();
    }

    public void resetAngle() {
        climbEncoder.setPosition(0);
    }

    public void climbReachGoal(double goalDegree) {
        pidController.setReference(goalDegree, ControlType.kPosition);
    }

    public Command setClimbGoal(double degree) {
        System.out.println("Setting turn goal to " + degree);
        return run(() -> climbReachGoal(degree));
    }

    /**
     * Sets the climber speed to 0
     */
    public void stopTurn() {
        climb.set(0.0);
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb position", getAngle());
    }
}
