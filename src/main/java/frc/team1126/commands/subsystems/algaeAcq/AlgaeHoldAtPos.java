package frc.team1126.commands.subsystems.algaeAcq;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.subsystems.AlgaeAcquisition;

public class AlgaeHoldAtPos extends Command{
    
    AlgaeAcquisition algaeAcquisition;
    double targetAngle;
    
    public AlgaeHoldAtPos(AlgaeAcquisition algaeAcquisition) {
        addRequirements(algaeAcquisition);
        this.algaeAcquisition = algaeAcquisition;
        targetAngle = algaeAcquisition.getAngle();
    }

    @Override
    public void execute() {
        algaeAcquisition.reachGoal(targetAngle);
    }

    @Override
    public boolean isFinished() {
        if(algaeAcquisition.getAngle() > 90) {
            return true;
        }
        return false;
    }
}
