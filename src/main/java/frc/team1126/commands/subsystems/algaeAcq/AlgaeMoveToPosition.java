package frc.team1126.commands.subsystems.algaeAcq;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.AlgaeAcquisition;

public class AlgaeMoveToPosition extends Command {

    AlgaeAcquisition algaeAcquisition;
    double targetAngle;
    
    public AlgaeMoveToPosition(AlgaeAcquisition algaeAcquisition, double angle) {
        addRequirements(RobotContainer.m_algae);
        this.algaeAcquisition = algaeAcquisition;
        targetAngle = angle;
    }

    @Override
    public void execute() {
        algaeAcquisition.reachGoal(targetAngle);
    }

    @Override
    public boolean isFinished() {
        return false;          
    }
}
