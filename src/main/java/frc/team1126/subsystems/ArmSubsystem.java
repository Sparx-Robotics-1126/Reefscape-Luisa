package frc.team1126.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1126.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private static final double MAX_ANGLE = 90.0;
    private static final double HOME_SPEED = -0.5;
    private static final double ARM_FEEDFORWARD_COEFFICIENT = 0.1;
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


    private RelativeEncoder turnEncoder;
    private RelativeEncoder extensionEncoder;

    private PIDController turnPID;
    private PIDController extensionPID;

    private SparkMaxConfig turnConfig;
    private SparkMaxConfig turn2Config;
    private SparkMaxConfig extensionConfig;

     protected ShuffleboardTab armTab;

     private GenericEntry kTurnPEntry;
     private GenericEntry kTurnIEntry;
     private GenericEntry kTurnDEntry;
     

//     ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0.1, 0.1);

    public ArmSubsystem() {
        turnMotor = new SparkMax(ArmConstants.TURN_ONE_ID, MotorType.kBrushless);
        turnFollower = new SparkMax(ArmConstants.TURN_TWO_ID, MotorType.kBrushless);
        turnController = turnMotor.getClosedLoopController();

        turnEncoder = turnMotor.getEncoder();

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
    private void configurePID() {
        double kTurnP = kTurnPEntry.getDouble(0);
        double kTurnI = kTurnIEntry.getDouble(0);
        double kTurnD = kTurnDEntry.getDouble(0);

        turnConfig.closedLoop
                .p(kTurnP)
                .i(kTurnI)
                .d(kTurnD)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    }
    /**
     * Add any extra SparkMax setting here.
     */
    private void configureSparkMaxes() {

//        extensionConfig.encoder
//                        .positionConversionFactor(Elevatore)


        turn2Config.follow(ArmConstants.TURN_ONE_ID);

        turnFollower.configure(turn2Config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        turnMotor.configure(turnConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    }

    private void initShuffleboard(){
        kTurnPEntry = armTab.add("Turn P", 0).getEntry();
        kTurnIEntry = armTab.add("Turn I", 0).getEntry();
        kTurnDEntry = armTab.add("Turn D", 0).getEntry();

    }


    public void angleToHome() {
        if(turnEncoder.getPosition() > 0) {
            turnMotor.set(HOME_SPEED);
        }
    }

    public void moveToAngle(double angle) {
        turnController.setReference(angle, ControlType.kPosition);
        double currentAngle = Math.max(getArmAngle(), 0);
        if (currentAngle < MAX_ANGLE) {
            double error = currentAngle - angle;
            double output = turnPID.calculate(error);
            double feedforward = ARM_FEEDFORWARD_COEFFICIENT * angle;
            turnMotor.set(output + feedforward);
        }
//
//        turnController.setReference(angle, ControlType.kMAXMotionPositionControl);
//
//        double targetAngle = angle;
//        double currentAngle = getArmAngle();
//
//        if(currentAngle < 0){
//            currentAngle = 0;
//        }
//
//        if (currentAngle < 90) {
//            double error =  currentAngle - targetAngle;
//            double output = turnPID.calculate(error);
//            double feedforward = 0.1 * targetAngle;
//
//            turnMotor.set(output + feedforward);
//        }
    }

    public double getArmAngle() {
        return turnEncoder.getPosition();
    }



   public void turnReachGoal(double goalDegree){
        turnController.setReference(Units.degreesToRotations(goalDegree),ControlType.kPosition);
   }

    public Command setTurnGoal(double degree){
        return run(() -> turnReachGoal(degree));
  }




public void stopTurn() {
    turnMotor.set(0.0);
  }

}
