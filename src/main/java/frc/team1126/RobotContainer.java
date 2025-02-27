// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1126;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team1126.Constants.OperatorConstants;
import frc.team1126.commands.drive.AbsoluteDriveAdv;
import frc.team1126.commands.drive.DriveToClosestLeftBranchPoseCommand;
import frc.team1126.commands.subsystems.LED.RainbowCommand;
import frc.team1126.commands.subsystems.LED.SetSolidColorCommand;
import frc.team1126.commands.subsystems.algaeAcq.AlgaeMoveToPosition;
import frc.team1126.commands.subsystems.arm.ControllerMoveArm;
import frc.team1126.commands.subsystems.arm.ControllerMoveExtension;
// import frc.team1126.commands.subsystems.algaeAcq.MoveAlgae;
import frc.team1126.commands.subsystems.arm.MoveArmToAngle;
import frc.team1126.commands.subsystems.arm.MoveExtHome;
import frc.team1126.commands.subsystems.arm.MoveExtensionToPos;
import frc.team1126.commands.subsystems.climb.ClimbMoveArm;
import frc.team1126.commands.subsystems.climb.ClimbMoveToPos;
import frc.team1126.commands.subsystems.climb.ClimbMoveUntil;
import frc.team1126.commands.subsystems.placer.AcquireCoral;
import frc.team1126.commands.subsystems.placer.AnalogPlacer;
import frc.team1126.commands.subsystems.placer.IngestCoral;
import frc.team1126.commands.subsystems.placer.PlaceCoral;
import frc.team1126.commands.subsystems.placer.PositionCoral;
// import frc.team1126.commands.subsystems.coralAcq.AcqMoveIn;
// import frc.team1126.commands.subsystems.coralAcq.AcqMoveOut;
import frc.team1126.subsystems.*;
import swervelib.SwerveInputStream;
import static edu.wpi.first.units.Units.Meter;

public class RobotContainer {

    //private final int m_rotationAxis = XboxController.Axis.kRightX.value;
    public static final ArmSubsystem m_arm = new ArmSubsystem();
     public static final ExtensionSubsystem m_extension = new ExtensionSubsystem();
    // public static final AlgaeAcquisition m_algae = new AlgaeAcquisition();

    public static final ClimbSubsystem m_climb = new ClimbSubsystem();

    public static final PlacerSubsystem m_placer = new PlacerSubsystem();

    final static SendableChooser<Command> m_chooser = new SendableChooser<>();

    public static CommandXboxController m_driver = new CommandXboxController(Constants.GeneralConstants.DRIVER_CONTROLLER_ID);
    public static CommandXboxController m_operator = new CommandXboxController(Constants.GeneralConstants.OPERATOR_CONTROLLER_ID);

    private final LEDs ledSubsystem = new LEDs(0, 200); //PORT IS PWM!!!

    public static SwerveSubsystem m_swerve = new SwerveSubsystem(
        new File(Filesystem.getDeployDirectory(), "swerve"));

 // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(m_swerve,
                                                                 () -> -MathUtil.applyDeadband(m_driver.getLeftY(),
                                                                                               OperatorConstants.LEFT_Y_DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(m_driver.getLeftX(),
                                                                                               OperatorConstants.DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(-m_driver.getRightX(),
                                                                                               OperatorConstants.RIGHT_X_DEADBAND),
                                                                                               m_driver.getHID()::getYButtonPressed,
                                                                                               m_driver.getHID()::getAButtonPressed,
                                                                                               m_driver.getHID()::getXButtonPressed,
                                                                                               m_driver.getHID()::getBButtonPressed);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swerve.getSwerveDrive(),
                                                                () -> m_driver.getLeftY() * -1,
                                                                () -> m_driver.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> m_driver.getRightX() * -1 )
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);
  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driver::getRightX,
  m_driver::getRightY)
                                                           .headingWhile(true);


    /**
     * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
     */
    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
            .allianceRelativeControl(false);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = m_swerve.driveFieldOriented(driveDirectAngle);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot


  Command driveFieldOrientedAnglularVelocity = m_swerve.driveFieldOriented(driveAngularVelocity);

  Command driveSetpointGen = m_swerve.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(m_swerve.getSwerveDrive(),
                                                                   () -> -m_driver.getLeftY(),
                                                                   () -> -m_driver.getLeftX())
                                                               .withControllerRotationAxis(() -> m_driver.getRawAxis(2))
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                        m_driver.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    m_driver.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);

    SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(m_swerve.getSwerveDrive(),
                    () -> -m_driver.getLeftY(),
                    () -> -m_driver.getLeftX())
            .withControllerRotationAxis(() -> m_driver.getRawAxis(
                    2))
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);
    // Derive the heading axis with math!
    SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
            .withControllerHeadingAxis(() ->
                            Math.sin(
                                m_driver.getRawAxis(
                                            2) *
                                            Math.PI) *
                                    (Math.PI *
                                            2),
                    () ->
                            Math.cos(
                                m_driver.getRawAxis(
                                            2) *
                                            Math.PI) *
                                    (Math.PI *
                                            2))
            .headingWhile(true);

  Command driveFieldOrientedDirectAngleSim = m_swerve.driveFieldOriented(driveDirectAngleSim);

  Command driveSetpointGenSim = m_swerve.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);
   
    public RobotContainer() {
      
        configurePathPlanner();

        // human control for climb and algae

        // m_climb.setDefaultCommand(new ClimbMoveArm(()-> m_operator.getRawAxis(XboxController.Axis.kLeftX.value), m_climb));
        m_arm.setDefaultCommand(new ControllerMoveArm(()-> m_operator.getRawAxis(XboxController.Axis.kLeftY.value), m_arm));
         //m_extension.setDefaultCommand(new ControllerMoveExtension(()-> m_operator.getRawAxis(XboxController.Axis.kRightY.value), m_extension));
         m_extension.setDefaultCommand(new MoveExtHome(m_extension, .05));

        ledSubsystem.setDefaultCommand(new RainbowCommand(ledSubsystem));


        // m_placer.setDefaultCommand(new AnalogPlacer(()-> m_operator.getRawAxis(XboxController.Axis.kLeftY.value), m_placer));

        // m_algae.setDefaultCommand(new MoveAlgae(m_algae, () -> m_operator.getRawAxis(XboxController.Axis.kRightY.value)));
       
        configureChooser();

        configureDriverBindings();
        configureOperatorBindings();
        DriverStation.silenceJoystickConnectionWarning(true);

    }

    private void configureDriverBindings() {
        
        Command driveFieldOrientedDirectAngle      = m_swerve.driveFieldOriented(driveDirectAngle);
        Command driveFieldOrientedAnglularVelocity = m_swerve.driveFieldOriented(driveAngularVelocity);
        Command driveRobotOrientedAngularVelocity  = m_swerve.driveFieldOriented(driveRobotOriented);
        Command driveSetpointGen = m_swerve.driveWithSetpointGeneratorFieldRelative(
                driveDirectAngle);
        Command driveFieldOrientedDirectAngleKeyboard      = m_swerve.driveFieldOriented(driveDirectAngleKeyboard);
        Command driveFieldOrientedAnglularVelocityKeyboard = m_swerve.driveFieldOriented(driveAngularVelocityKeyboard);
        Command driveSetpointGenKeyboard = m_swerve.driveWithSetpointGeneratorFieldRelative(                driveDirectAngleKeyboard);

        m_swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);
 if (RobotBase.isSimulation())
        {
            m_swerve.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
        } else
        {
            m_swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        }


       if (DriverStation.isTest())
        {
            m_swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

            // m_driver.x().whileTrue(Commands.runOnce(m_swerve::lock, m_swerve).repeatedly());
            // m_driver.y().whileTrue(m_swerve.driveToDistanceCommand(1.0, 0.2));
            m_driver.start().onTrue((Commands.runOnce(m_swerve::zeroGyro)));
            m_driver.back().whileTrue(m_swerve.centerModulesCommand());
            m_driver.leftBumper().onTrue(Commands.none());
            m_driver.rightBumper().onTrue(Commands.none());
        } else {
            m_driver.leftTrigger().onTrue(new InstantCommand(() -> m_swerve.zeroGyro()));
            m_driver.rightTrigger().onChange(new InstantCommand(() -> m_swerve.zeroGyroWithAlliance()));
            m_driver.a().onTrue((Commands.runOnce(m_swerve::zeroGyro)));
            m_driver.y().whileTrue(new ClimbMoveToPos(m_climb, 0));
            m_driver.x().whileTrue(new ClimbMoveToPos(m_climb, 125));
            m_driver.b().whileTrue(new ClimbMoveToPos(m_climb, -120));
            //m_driver.b().whileTrue(new IngestCoral(m_placer));




//        m_driver.x().onTrue(Commands.runOnce(m_swerve::addFakeVisionReading));
            // m_driver.b().whileTrue(
            //     m_swerve.driveToPose(
            //       new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
            //                       );
            //                     //   m_driver.y().whileTrue(m_swerve.aimAtSpeaker(2));
            //                       m_driver.start().whileTrue(Commands.none());
            //                       m_driver.back().whileTrue(Commands.none());
            //                       m_driver.leftBumper().whileTrue(Commands.runOnce(m_swerve::lock, m_swerve).repeatedly());

            //                       m_driver.rightBumper().onTrue(Commands.none());
            // m_driver.y().onTrue(new ChaseLEDColorCommand(ledSubsystem, new Color8Bit(255, 0, 0), 10)); // Chasing red color
            // m_driver.x().onTrue(new GradientCommand(ledSubsystem, new Color8Bit(0,0,255), new Color8Bit(255,0,0)));
            // m_driver.a().onTrue(new RainbowCommand(ledSubsystem));
            // m_driver.b().onTrue(new PulseCommand(ledSubsystem, new Color8Bit(0, 255, 0), 7));
            //m_driver.leftBumper().onTrue(new SetSolidColorCommand(ledSubsystem, new Color8Bit(0,0,255)));
            m_driver.leftBumper().whileTrue(new DriveToClosestLeftBranchPoseCommand(m_swerve));
            // m_driver.x().whileTrue(new LinearDriveToPose(m_swerve, () -> m_swerve.getClosestRightBranchPose(), () -> new ChassisSpeeds()));
            //  driverController.x().whileTrue(new DriveToAprilTagCommand(swerve, m_noteCamera, driverController.getHID()));
            // m_driver.b().whileTrue(m_swerve.driveToPose(new Pose2d(new Translation2d(Meter.of(16.4), Meter.of(4.4)), Rotation2d.fromDegrees(180))));
            // m_driver.y().whileTrue(m_swerve.driveToPose(new Pose2d(new Translation2d(13, 4), Rotation2d.fromDegrees(180))));
            m_driver.leftBumper().whileTrue(m_swerve.driveToPose(m_swerve.getClosestLeftBranchPose()));
            // driverController.leftBumper().whileTrue(new LinearDriveToPose(swerve, () -> swerve.getClosestLeftBranchPose(),() ->  new ChassisSpeeds()));
            // driverController.rightBumper().whileTrue(new LinearDriveToPose(swerve, () -> swerve.getClosestRightBranchPose(), () -> new ChassisSpeeds()));
        }
    }

    public void configureOperatorBindings() {   

       // WE NEED TO MAKE L3 HIGHER THAN 24.45223045349121 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        m_operator.povDown().whileTrue(new MoveArmToAngle(m_arm, -.01).alongWith(new MoveExtensionToPos(m_extension, m_arm, 0.01))); //arm home
        m_operator.povUp().whileTrue(new MoveArmToAngle(m_arm, 17.642849922180176).alongWith(new MoveExtensionToPos(m_extension, m_arm, .01))
                   .alongWith(new IngestCoral(m_placer).andThen(new PositionCoral(m_placer))));                                                         //arm to coral station

        m_operator.a().whileTrue(new MoveArmToAngle(m_arm, 11.76196).alongWith(new MoveExtensionToPos(m_extension,m_arm, 0.013659))); //arm l1
        m_operator.x().whileTrue(new MoveArmToAngle(m_arm, 22.238).alongWith(new MoveExtensionToPos(m_extension, m_arm,-0.0831989))); //arm l2
        m_operator.b().whileTrue(new MoveArmToAngle(m_arm,  26.5).alongWith(new MoveExtensionToPos(m_extension, m_arm, -0.25))); //arm l3
        m_operator.y().whileTrue(new MoveArmToAngle(m_arm, 33.5).alongWith(new MoveExtensionToPos(m_extension, m_arm, -0.55))); //arm l4

        // m_operator.rightBumper().whileTrue(new AlgaeMoveToPosition(m_algae, 5)); //move out
        // m_operator.leftBumper().whileTrue(new AlgaeMoveToPosition(m_algae, 0)); // move home

        m_operator.rightTrigger(0.1).whileTrue(new AnalogPlacer(() -> m_operator.getRawAxis(XboxController.Axis.kRightTrigger.value), m_placer,false));
        m_operator.leftTrigger(0.1).whileTrue(new AnalogPlacer(() -> m_operator.getRawAxis(XboxController.Axis.kLeftTrigger.value), m_placer,true));

    }
   

    double getXSpeed() {
        int pov = m_driver.getHID().getPOV();
        double finalX;

        if (pov == 0)
            finalX = -0.05;
        else if (pov == 180)
            finalX = 0.05;
        else if (Math.abs(m_driver.getLeftY()) <= 0.1)
            finalX = 0.0;
        else
            finalX = m_driver.getLeftY() * 0.75 * (1.0 + m_driver.getLeftTriggerAxis());

        return finalX;
    }

    public double getYSpeed() {
        int pov = m_driver.getHID().getPOV();

        double finalY;
        if (pov == 270 || pov == 315 || pov == 225)
            finalY = -0.05;
        else if (pov == 90 || pov == 45 || pov == 135)
            finalY = 0.05;
        else if (Math.abs(m_driver.getLeftX()) <= 0.1)
            finalY = 0.0;
        else
            finalY = m_driver.getLeftX() * 0.75 * (1.0 + m_driver.getLeftTriggerAxis());

        return finalY;
    }

    public void configureChooser() {
        // autos using pathplanner
        m_chooser.setDefaultOption("Do Nothing", new WaitCommand(15));
        m_chooser.addOption("3MeterTest", new PathPlannerAuto("3MeterTest"));

    }
    
    /* REGISTER PATHPLANNER COMMANDS HERE */
    public void configurePathPlanner() {
        
        NamedCommands.registerCommand("Wait", new WaitCommand(1));

        NamedCommands.registerCommand("MoveArmToHome", new MoveArmToAngle(m_arm, 0).alongWith(new MoveExtensionToPos(m_extension,m_arm, 0)));
        NamedCommands.registerCommand("MoveArmToCoral", new MoveArmToAngle(m_arm, 0).alongWith(new MoveExtensionToPos(m_extension, m_arm, 0)));
        NamedCommands.registerCommand("MoveArmToL1", new MoveArmToAngle(m_arm, 11.76196).alongWith(new MoveExtensionToPos(m_extension,m_arm, 0).withTimeout(1).andThen(new WaitCommand(1).andThen(new MoveExtensionToPos(m_extension,m_arm, 0.013659)))).withTimeout(1));
        NamedCommands.registerCommand("MoveArmToL2", new MoveArmToAngle(m_arm, 22.238).alongWith(new MoveExtensionToPos(m_extension,m_arm, 0).withTimeout(1).andThen(new WaitCommand(1).andThen(new MoveExtensionToPos(m_extension,m_arm, -0.0831989)))).withTimeout(1));
        NamedCommands.registerCommand("MoveArmToL3", new MoveArmToAngle(m_arm, 25.1665).alongWith(new MoveExtensionToPos(m_extension,m_arm, 0).withTimeout(1).andThen(new WaitCommand(1).andThen(new MoveExtensionToPos(m_extension,m_arm, -0.1801146)))).withTimeout(1));
        NamedCommands.registerCommand("MoveArmToL4", new MoveArmToAngle(m_arm, 32).alongWith(new MoveExtensionToPos(m_extension,m_arm, 0).withTimeout(1).andThen(new WaitCommand(1).andThen(new MoveExtensionToPos(m_extension,m_arm, -0.45723746)))).withTimeout(1));

        NamedCommands.registerCommand("SpinPlacerOut", new PlaceCoral(m_placer).withTimeout(1));
        NamedCommands.registerCommand("SpinPlacerIn",new AcquireCoral(m_placer).withTimeout(1));

        NamedCommands.registerCommand("IngestCoral", new IngestCoral(m_placer));

        

   }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *\
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // // the command to be run in autonomous

        // return _chooser.getSelected();
        return m_chooser.getSelected();
        // return swerve.getAutonomousCommand(_chooser.().getName(), true);

    }

    // public static void getSmartDashboardTable() {
    //     ShuffleboardTab tab = Shuffleboard.getTab("SmartDashboard");
    //     GenericEntry test = tab.add("test variable", 1).getEntry();
    // }


}
