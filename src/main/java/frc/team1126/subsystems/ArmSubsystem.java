package frc.team1126.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1126.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private static final double MAX_ANGLE = 90.0;
    private static final double HOME_SPEED = -0.5;
    private static final double ARM_FEEDFORWARD_COEFFICIENT = 0.1;

    /**
     * Subsystem-wide setpoints
     */
    public enum Setpoint {
        kFeederStation,
        kLevel1,
        kLevel2,
        kLevel3,
        kLevel4;
    }

    // private final MutAngle mutAngle = Rotations.mutable(0);
    private SparkMax turnMotor;
    private SparkMax turnFollower; // follows turnMotor
    private SparkClosedLoopController turnController;

    private DigitalInput homeSensor;


    private RelativeEncoder turnEncoder;

    private PIDController turnPID;

    private SparkMaxConfig turnConfig;
    private SparkMaxConfig turn2Config;

    protected ShuffleboardTab armTab;

    private GenericEntry kTurnPEntry;
    private GenericEntry kTurnIEntry;
    private GenericEntry kTurnDEntry;

    private double kP, kI, kD = 0;
    private double angle = 0;

    ElevatorFeedforward m_feedforward =
    new ElevatorFeedforward(
       0.0,
       .762,
        .762,
       0);

//     ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0.1, 0.1);

    public ArmSubsystem() {
          if (!RobotBase.isSimulation()){

        turnMotor = new SparkMax(ArmConstants.TURN_ONE_ID, MotorType.kBrushless);
        turnFollower = new SparkMax(ArmConstants.TURN_TWO_ID, MotorType.kBrushless);
        
        turnController = turnMotor.getClosedLoopController();

        turnEncoder = turnMotor.getEncoder();

        turnConfig = new SparkMaxConfig();

        homeSensor = new DigitalInput(ArmConstants.ARM_SENSOR_ID);

        turn2Config = new SparkMaxConfig();
        armTab = Shuffleboard.getTab("ArmTab");

        initShuffleboard();
        configurePID();
        configureSparkMaxes();

          }
    }

    /*
     * configure PID settings here.
     */
    private void configurePID() {
        double kTurnP = kTurnPEntry.getDouble(0);
        double kTurnI = kTurnIEntry.getDouble(0);
        double kTurnD = kTurnDEntry.getDouble(0);


        var p=.03;
        turnConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(p)
                .i(kTurnI)
                .d(kTurnD)
                .outputRange(-1, 1)
                .velocityFF(1.0/5767);
                // .minOutput(-.04).maxOutput(.06)

                turn2Config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(p)
                .i(kTurnI)
                .d(kTurnD)
                .outputRange(-1, 1)
                .velocityFF(1.0/5767);

    }

    /**
     * Add any extra SparkMax setting here.
     */
    private void configureSparkMaxes() {

//        extensionConfig.encoder
//                        .positionConversionFactor(Elevatore)


        turn2Config.follow(ArmConstants.TURN_ONE_ID);

        turnFollower.configure(turn2Config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        turnMotor.configure(turnConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

    }

    private void initShuffleboard() {
        
        // armTab = Shuffleboard.getTab("ArmTab");
        kTurnPEntry = armTab.add("Turn P", 0).getEntry();
        kTurnIEntry = armTab.add("Turn I", 0).getEntry();
        kTurnDEntry = armTab.add("Turn D", 0).getEntry();
        angle = armTab.add("Angle", 0).getEntry().getDouble(0);
        // armTab.add("Turn Command", setTurnGoal(angle));
//   SmartDashboard.putData("Turn Command", new InstantCommand(() -> setTurnGoal(angle)));
    }


    // public void angleToHome() {
    //     if (turnEncoder.getPosition() > 0) {
    //         turnMotor.set(HOME_SPEED);
    //     }
    // }

    public void moveArm(double speed) {
        // System.out.println("ddddd");
        turnMotor.set(speed);
    }

    // public void moveToAngle(double angle) {
    //     turnController.setReference(angle, ControlType.kPosition);
    //     double currentAngle = Math.max(getArmAngle(), 0);
    //     if (currentAngle < MAX_ANGLE) {
    //         double error = currentAngle - angle;
    //         double output = turnPID.calculate(error);
    //         double feedforward = ARM_FEEDFORWARD_COEFFICIENT * angle;
    //         turnMotor.set(output + feedforward);
    //     }
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
    // }
    /**
     * Returns the position of the arm
     */
    public double getArmAngle() {
        return turnEncoder.getPosition();
    }
    
    public void turnReachGoal(double goalDegree) {
        // System.out.println("In here " + goalDegree);
        turnController.setReference(goalDegree, ControlType.kPosition);
    }

    public Command setTurnGoal(double degree) {
        // System.out.println("Setting turn goal to " + degree);
        return run(() -> turnReachGoal(degree));
    }

    /**
     * Sets arm speed to 0
     */
    public void stopTurn() {
        turnMotor.set(0.0);
    }

    public boolean isHome(){
        return !homeSensor.get();
    }

    @Override
    public void periodic() {
        if (!RobotBase.isSimulation()){

        // turnConfig.closedLoop.pid(kTurnPEntry.getDouble(0), kTurnIEntry.getDouble(0), kTurnDEntry.getDouble(0));
        // var p = kTurnPEntry.getDouble(0);
        // var i = kTurnIEntry.getDouble(0);
        // var d = kTurnDEntry.getDouble(0);
        // var ddd = turnMotor.getAbsoluteEncoder();
        // ddd.getPosition();

        // // armTab.add("Current Position", turnEncoder.getPosition());
        // if (p != kP || i != kI || d != kD) {

        //     turnConfig.closedLoop
        //             .p(p)
        //             .i(i)
        //             .d(d)
        //             .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        //     turnMotor.configure(turnConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        //     kP = p;
        //     kI = i;
        //     kD = d;
        // }
        SmartDashboard.putNumber("Arm position", getArmAngle());
    }
    }

}
