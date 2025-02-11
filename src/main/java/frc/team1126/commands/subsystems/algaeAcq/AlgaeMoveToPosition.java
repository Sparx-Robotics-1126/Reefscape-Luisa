package frc.team1126.commands.subsystems.algaeAcq;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.subsystems.AlgaeAcquisition;

public class AlgaeMoveToPosition extends Command {

    AlgaeAcquisition algaeAcquisition;
    double targetAngle;
    
    public AlgaeMoveToPosition(AlgaeAcquisition algaeAcquisition, double angle) {
        addRequirements(algaeAcquisition);
        this.algaeAcquisition = algaeAcquisition;
        targetAngle = angle;
    }

    @Override
    public void execute() {
        algaeAcquisition.moveToAngle(targetAngle);
    }

    @Override
    public boolean isFinished() {
        if(algaeAcquisition.getAngle() > 90) {
            return true;
        }
        return false;
    }
}
