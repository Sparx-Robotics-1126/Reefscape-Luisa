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

//     ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0.1, 0.1);

    public ArmSubsystem() {
        turnMotor = new SparkMax(ArmConstants.TURN_ONE_ID, MotorType.kBrushless);
        turnFollower = new SparkMax(ArmConstants.TURN_TWO_ID, MotorType.kBrushless);
        turnController = turnMotor.getClosedLoopController();

        turnEncoder = turnMotor.getEncoder();

        turnConfig = new SparkMaxConfig();


        turn2Config = new SparkMaxConfig();
        armTab = Shuffleboard.getTab("ArmTab");

        initShuffleboard();
        configurePID();
        configureSparkMaxes();


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

    private void initShuffleboard() {
        // armTab = Shuffleboard.getTab("ArmTab");
        kTurnPEntry = armTab.add("Turn P", 0).getEntry();
        kTurnIEntry = armTab.add("Turn I", 0).getEntry();
        kTurnDEntry = armTab.add("Turn D", 0).getEntry();
        angle = armTab.add("Angle", 0).getEntry().getDouble(0);
        // armTab.add("Turn Command", setTurnGoal(angle));
//   SmartDashboard.putData("Turn Command", new InstantCommand(() -> setTurnGoal(angle)));
    }


    public void angleToHome() {
        if (turnEncoder.getPosition() > 0) {
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
    /**
     * Returns the position of the arm
     */
    public double getArmAngle() {
        return turnEncoder.getPosition();
    }
    
    public void turnReachGoal(double goalDegree) {
        turnController.setReference(Units.degreesToRotations(goalDegree), ControlType.kPosition);
    }

    public Command setTurnGoal(double degree) {
        System.out.println("Setting turn goal to " + degree);
        return run(() -> turnReachGoal(degree));
    }

    /**
     * Sets arm speed to 0
     */
    public void stopTurn() {
        turnMotor.set(0.0);
    }

    @Override
    public void periodic() {

        // turnConfig.closedLoop.pid(kTurnPEntry.getDouble(0), kTurnIEntry.getDouble(0), kTurnDEntry.getDouble(0));
        var p = kTurnPEntry.getDouble(0);
        var i = kTurnIEntry.getDouble(0);
        var d = kTurnDEntry.getDouble(0);
        var ddd = turnMotor.getAbsoluteEncoder();
        ddd.getPosition();

        armTab.add("Current Position", turnEncoder.getPosition());
        if (p != kP || i != kI || d != kD) {

            turnConfig.closedLoop
                    .p(p)
                    .i(i)
                    .d(d)
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

            turnMotor.configure(turnConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
            kP = p;
            kI = i;
            kD = d;
        }

    }

}
