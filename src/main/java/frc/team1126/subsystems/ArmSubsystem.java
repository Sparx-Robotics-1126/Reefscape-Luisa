package frc.team1126.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1126.Constants.ArmConstants;
import frc.team1126.Constants.CoralSubsystemConstants;
import frc.team1126.Constants.CoralSubsystemConstants.ArmSetpoints;
import frc.team1126.Constants.CoralSubsystemConstants.ElevatorSetpoints;
import frc.team1126.Constants.CoralSubsystemConstants.IntakeSetpoints;

public class ArmSubsystem extends SubsystemBase {
      /** Subsystem-wide setpoints */
  public enum Setpoint {
    kFeederStation,
    kLevel1,
    kLevel2,
    kLevel3,
    kLevel4;
  }
    private SparkMax turnMotor;
    private SparkMax turnFollower; // follows turnMotor
    private SparkClosedLoopController turnController;

    private SparkMax extension;
    private SparkClosedLoopController extensionController;

    private SparkAbsoluteEncoder turnEncoder;
    private SparkAbsoluteEncoder extensionEncoder;

    private PIDController turnPID;
    private PIDController extensionPID;

    private SparkMaxConfig turnConfig;
    private SparkMaxConfig turn2Config;
    private SparkMaxConfig extensionConfig;

     protected ShuffleboardTab armTab;

     private double kTurnP;
     private double kTurnI;
     private double kTurnD;
     private double kTurnMinOutput;
     private double kTurnMaxOutput;
    //  private double kTurnFF = 473 ;
     private double kTurnMaxVel;
     private double kTurnMaxAccel;


     private double kExtP;
     private double kExtI;
     private double kExtD;
     private double kExtMinOutput;
     private double kExtMaxOutput;
    //  private double kExtFF = 473 ;
     private double kExtMaxVel;
     private double kExtMaxAccel;

     private GenericEntry kTurnPEntry;
     private GenericEntry kTurnIEntry;
     private GenericEntry kTurnDEntry;
     
     private GenericEntry kExtPEntry;
     private GenericEntry kExtIEntry;
     private GenericEntry kExtDEntry;

    public ArmSubsystem() {

        turnMotor = new SparkMax(ArmConstants.TURN_ONE_ID, MotorType.kBrushless);
        turnFollower = new SparkMax(ArmConstants.TURN_TWO_ID, MotorType.kBrushless);
        turnController = turnMotor.getClosedLoopController();

        extension = new SparkMax(ArmConstants.ELEVATOR_ID, MotorType.kBrushless);
        extensionController = extension.getClosedLoopController();

        turnEncoder = turnMotor.getAbsoluteEncoder();
        extensionEncoder = extension.getAbsoluteEncoder();

        turnConfig = new SparkMaxConfig();
       

        // turn2Config = new SparkMaxConfig();
        extensionConfig = new SparkMaxConfig();
       

        initShuffleboard();
        configurePID();
        configureSparkMaxes();
        armTab = Shuffleboard.getTab("ArmTab");
       
    }

    /*
     * configure PID settings here.
     */
    private void configurePID(){
        kTurnP = kTurnPEntry.getDouble(0);
        kTurnI = kTurnIEntry.getDouble(0);
        kTurnD = kTurnDEntry.getDouble(0);

        kExtP = kExtPEntry.getDouble(0);
        kExtI = kExtIEntry.getDouble(0);
        kExtD = kExtDEntry.getDouble(0);

        turnConfig.closedLoop
        .p(kTurnP)
        .i(kTurnI)
        .d(kTurnD)
        // .outputRange(kTurnMinOutput, kTurnMaxOutput)
        // .velocityFF(kTurnFF)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        // .maxMotion.maxVelocity(kTurnMaxVel)
                // .maxAcceleration(kTurnMaxAccel);

        extensionConfig.closedLoop
                .p(kExtP)
                .i(kExtI)
                .d(kExtD)
                // .outputRange(kExtMinOutput, kExtMaxOutput)
                // .velocityFF(kExtFF)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
                // .maxMotion.maxVelocity(kExtMaxVel)
                // .maxAcceleration(kExtMaxAccel);
        // turnPID = new PIDController(0, 0, 0);
        // extensionPID = new PIDController(0, 0, 0);
    }

    /**
     * Add any extra SparkMax setting here.
     */
    private void configureSparkMaxes() {
       
        turn2Config.follow(ArmConstants.TURN_ONE_ID);

        turnFollower.configure(turn2Config, null, null);
        turnMotor.configure(turnConfig, null, null);
        extension.configure(extensionConfig, null, null);
    
    }

    private void initShuffleboard(){
        kTurnPEntry = armTab.add("Turn P,", kTurnP).getEntry();
        kTurnIEntry = armTab.add("Turn I", kTurnI).getEntry();
        kTurnDEntry = armTab.add("Turn D", kTurnD).getEntry();

        kExtPEntry = armTab.add("Ext P,", kExtP).getEntry();
        kExtIEntry = armTab.add("Ext I", kExtI).getEntry();
        kExtDEntry = armTab.add("Ext D", kExtD).getEntry();
    }

    public void moveToExtension(double position) {
        double targetPos = position;
        double currentPos = getExtension();

        if(currentPos < 0){
            currentPos = 0;
        }
    
        if (currentPos < 90) {
            double error =  currentPos - targetPos;
            double output = extensionPID.calculate(error);
            double feedforward = 0.1 * targetPos;

            extension.set(output + feedforward);
        }
    }

    public void extensionToHome() {
        if(extensionEncoder.getPosition() > 0) {
            extension.set(-0.5);
        }
    }
   
    public void angleToHome() {
        if(turnEncoder.getPosition() > 0) {
            turnMotor.set(-0.5);
        }
    }

    public void moveToAngle(double angle) {
        turnController.setReference(angle, ControlType.kMAXMotionPositionControl);

        double targetAngle = angle;
        double currentAngle = getArmAngle();

        if(currentAngle < 0){
            currentAngle = 0;
        }
    
        if (currentAngle < 90) {
            double error =  currentAngle - targetAngle;
            double output = turnPID.calculate(error);
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

   public void turnReachGoal(double goalDegree){
        turnController.setReference(Units.degreesToRotations(goalDegree),ControlType.kPosition);
   }

    public Command setTurnGoal(double degree){
        return run(() -> turnReachGoal(degree));
  }

  public void extReachGoal(double goalDegree){
    turnController.setReference(Units.degreesToRotations(goalDegree),ControlType.kPosition);
}

public Command setExtGoal(double degree){
    return run(() -> extReachGoal(degree));
}

public void stopTurn() {
    turnMotor.set(0.0);
  }
  public void stopExtension() {
    extension.set(0.0);
  }
}
