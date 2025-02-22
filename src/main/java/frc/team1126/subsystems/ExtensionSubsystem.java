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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1126.Constants.ArmConstants;

public class ExtensionSubsystem extends SubsystemBase {
    private static final double MAX_EXTENSION = 90.0;
    private static final double HOME_SPEED = -0.5;
    private static final double EXTENSION_FEEDFORWARD_COEFFICIENT = 0.1;
    /** Subsystem-wide setpoints */
    public enum Setpoint {
        kFeederStation,
        kLevel1,
        kLevel2,
        kLevel3,
        kLevel4;
    }

    private SparkMax extension;
    private SparkClosedLoopController extensionController;

    private RelativeEncoder extensionEncoder;

    private PIDController extensionPID;

    private SparkMaxConfig extensionConfig;

    private DigitalInput homeSensor;

    protected ShuffleboardTab armTab;

    private GenericEntry kExtPEntry;
    private GenericEntry kExtIEntry;
    private GenericEntry kExtDEntry;

    private double kElevatorkS= 0.0;
    private double kElevatorkG = .762;
    private double kElevatorkV =.762 ;
    private double kElevatorkA = 0.0;

    private double kP, kI, kD = 0;
    private double angle;


    ElevatorFeedforward m_feedforward =
            new ElevatorFeedforward(
                    kElevatorkS,
                    kElevatorkG,
                    kElevatorkV,
                    kElevatorkA);

    public ExtensionSubsystem() {
  if (!RobotBase.isSimulation()){

        extension = new SparkMax(ArmConstants.ELEVATOR_ID, MotorType.kBrushless);
        extensionController = extension.getClosedLoopController();

        extensionEncoder = extension.getEncoder();

        extensionConfig = new SparkMaxConfig();

        homeSensor = new DigitalInput(ArmConstants.EXTENSION_SENSOR_ID);

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

        double kExtP = kExtPEntry.getDouble(0);
        double kExtI = kExtIEntry.getDouble(0);
        double kExtD = kExtDEntry.getDouble(0);

        extensionConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(3.1)
                .i(kExtI)
                .d(kExtD)
                .outputRange(-.25, .25)
                .velocityFF(1.0/5767);
    }
    
    /**
     * Add any extra SparkMax setting here.
     */
    private void configureSparkMaxes() {

//        extensionConfig.encoder
//                        .positionConversionFactor(Elevatore)

        extension.configure(extensionConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    }

    private void initShuffleboard(){
        kExtPEntry = armTab.add("Ext P", 0).getEntry();
        kExtIEntry = armTab.add("Ext I", 0).getEntry();
        kExtDEntry = armTab.add("Ext D", 0).getEntry();

        // angle = armTab.add("Angle", 0).getEntry().getDouble(0);
    }

    /**
     * Moves the extension to a specific position
     * @param position the position to move the extension to
     */
    public void moveExtensionToPosition(double position) {

        double currentPos = Math.max(getExtension(), 0);
        if (currentPos < MAX_EXTENSION) {
            double error = currentPos - position;
            double output = extensionPID.calculate(error);
            double feedforward = EXTENSION_FEEDFORWARD_COEFFICIENT * position;
            extension.set(output + feedforward);
        }
    }

    /**
     * Moves the extension to home
     */
    public void extensionToHome() {
        if(extensionEncoder.getPosition() > 0) {
            extension.set(-0.5);
        }
    }

    /**
     * Returns the current position of the extension 
     */
    public double getExtension() {
        return extensionEncoder.getPosition();
    }

    public void extReachGoal(double goalDistance){
        extensionController.setReference(goalDistance, ControlType.kPosition, ClosedLoopSlot.kSlot0, m_feedforward.calculate(goalDistance));
    }

    public Command setExtGoal(double distance){
        return run(() -> extReachGoal(distance));
    }

    /**
     * Sets the speed of the extension to 0
     */
    public void stopExtension() {
        extension.set(0.0);
    }

    public boolean isExtensionHome(){
        return !homeSensor.get();
    }


    public void moveExtension(double speed){
        extension.set(speed);
    }

    @Override
    public void periodic() {
        if (!RobotBase.isSimulation()){

        // var p = kExtPEntry.getDouble(0);
        // var i = kExtIEntry.getDouble(0);
        // var d = kExtDEntry.getDouble(0);
        // var ddd = extension.getAbsoluteEncoder();
        // ddd.getPosition();

        // // armTab.add("Current Position", extensionEncoder.getPosition());
        // if (p != kP || i != kI || d != kD) {

        //     extensionConfig.closedLoop
        //             .p(p)
        //             .i(i)
        //             .d(d)
        //             .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        //     extension.configure(extensionConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        //     kP = p;
        //     kI = i;
        //     kD = d;
        // }
        SmartDashboard.putNumber("Extension Position", getExtension());
    }
       
    }
}
