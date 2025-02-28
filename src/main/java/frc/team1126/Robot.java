// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1126;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team1126.commands.EndGameRumble;
import frc.team1126.subsystems.LEDs;

public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private EndGameRumble rumble = new EndGameRumble(RobotContainer.m_driver);
    public static RobotContainer robotContainer;
    public static int ledColor;

    @Override
    public void robotInit() {
        enableLiveWindowInTest(true);
        robotContainer = new RobotContainer();
        // Autos.init();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        // RobotContainer.getSmartDashboardTable();
        
        SmartDashboard.putNumber("Controller speed", RobotContainer.m_operator.getLeftY());
        SmartDashboard.putData("AUTO CHOICES ", RobotContainer.m_chooser);
    }

    @Override
    public void disabledInit() {
        RobotContainer.ledSubsystem.setAllianceColorCommand();
        
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
        //rumble.execute();
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
