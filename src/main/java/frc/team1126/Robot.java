// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1126;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command autonomousCommand;
    public static RobotContainer robotContainer;
    public static int ledColor;
    // public static final CANdleSubsystem m_candleSubsystem = new CANdleSubsystem();

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        // Autos.init();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        

        SmartDashboard.putData("AUTO CHOICES ", RobotContainer.m_chooser);
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        if (RobotBase.isReal())
        {
            
        }
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        // robotContainer.setMotorBrake(true);
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null)
            autonomousCommand.cancel();
    }

    @Override
    public void teleopPeriodic() {
        robotContainer.EndGameRumble();
        robotContainer.upToSpeedRumble();

    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
