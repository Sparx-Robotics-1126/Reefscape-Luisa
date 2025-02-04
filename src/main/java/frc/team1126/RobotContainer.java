// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1126;

import java.io.File;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team1126.Constants.OperatorConstants;
import frc.team1126.commands.drive.AbsoluteDriveAdv;
import frc.team1126.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

    private final int m_rotationAxis = XboxController.Axis.kRightX.value;

    // public static final CANdleSubsystem m_candleSubsystem = new CANdleSubsystem();

    final static SendableChooser<Command> m_chooser = new SendableChooser<>();

    CommandXboxController m_driver = new CommandXboxController(Constants.GeneralConstants.DRIVER_CONTROLLER_ID);
    // CommandXboxController m_operator = new CommandXboxController(Constants.GeneralConstants.OPERATOR_CONTROLLER_ID);

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
                                                                 () -> -MathUtil.applyDeadband(m_driver.getRightX(),
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
                                                            .withControllerRotationAxis(m_driver::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);
  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driver::getRightX,
  m_driver::getRightY)
                                                           .headingWhile(true);


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

  Command driveFieldOrientedDirectAngleSim = m_swerve.driveFieldOriented(driveDirectAngleSim);

  Command driveSetpointGenSim = m_swerve.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);
   
    public RobotContainer() {
      
        /* REGISTER PATHPLANNER COMMANDS HERE */
     
    // Command test = m_swerve.driveCommand(
    //             () -> MathUtil.clamp(MathUtil.applyDeadband(m_driver.getLeftY(), .1), -1,
    //                     1),
    //             () -> MathUtil.clamp(MathUtil.applyDeadband(m_driver.getLeftX(), .1), -1,
    //                     1),
    //             () -> m_driver.getRightX());

        //OTHER COMMANDS
        //NamedCommands.registerCommand("limelightTarget", new LLRotationAlignCommand(m_swerve).withTimeout(1.5));
        //DriverStation.silenceJoystickConnectionWarning(true);

        // m_swerve.setDefaultCommand(test);
        // m_swerve = new SwerveSubsystem(
                // new File(Filesystem.getDeployDirectory(), "swerve"));

        Command driveFieldOrientedAnglularVelocity = m_swerve.driveCommand(
                () -> MathUtil.clamp(MathUtil.applyDeadband(-m_driver.getLeftY()*.75, .1), -1,
                        1),
                () -> MathUtil.clamp(MathUtil.applyDeadband(-m_driver.getLeftX()*.75, .1), -1,
                        1),
                () -> -m_driver.getRightX());

        m_swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);
       
        // configureChooser();

        configureDriverBindings();

    }

    private void configureDriverBindings() {
        
        

        m_driver.leftTrigger().onTrue(new InstantCommand(() -> m_swerve.zeroGyro()));
        // m_driver.a().onTrue((Commands.runOnce(m_swerve::zeroGyro)));
        // m_driver.x().onTrue(Commands.runOnce(m_swerve::addFakeVisionReading));
        // m_driver.b().whileTrue(
        //     m_swerve.driveToPose(
        //       new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
        //                       );
        //                       m_driver.y().whileTrue(m_swerve.aimAtSpeaker(2));
        //                       m_driver.start().whileTrue(Commands.none());
        //                       m_driver.back().whileTrue(Commands.none());
        //                       m_driver.leftBumper().whileTrue(Commands.runOnce(m_swerve::lock, m_swerve).repeatedly());
        //                       m_driver.rightBumper().onTrue(Commands.none());
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

 

    public void EndGameRumble() {
        if (DriverStation.getMatchTime() < Constants.SwerveConstants.ENDGAME_SECONDS 
                && DriverStation.getMatchTime() > Constants.SwerveConstants.STOP_RUMBLE_SECONDS) {
            
            m_driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1);
            //m_operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1);

        } else {
            m_driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
            //m_operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);

        }
    }

    //method for operator to know whether or not the shooter is up to speed for any given distance
    public void upToSpeedRumble() {
        // if(m_shooter.isMotorUpToSpeed()) {
        //     m_operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble,1);
        // } 
        // m_operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble,0);
    }

}
