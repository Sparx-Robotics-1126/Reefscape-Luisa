package frc.team1126.commands.subsystems.placer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.PlacerSubsystem;

public class PositionCoral extends Command {

    private PlacerSubsystem placer;

    public PositionCoral(PlacerSubsystem placer) {
        addRequirements(RobotContainer.m_placer);
        this.placer = placer;
    }

    @Override
    public void execute() {
        // if(placer.bottomHasCoral()) {
            placer.movePlacer(-.25);
        // }
    //     else {
    //         placer.movePlacer(0);
    //     }
     }

    @Override
    public boolean isFinished() {

        if(!placer.bottomHasCoral()){
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        placer.movePlacer(0);
    }
    
}
