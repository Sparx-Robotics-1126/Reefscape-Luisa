package frc.team1126.commands.subsystems.placer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.subsystems.PlacerSubsystem;

public class AcquireCoral extends Command {
   
    private PlacerSubsystem placer;

    public AcquireCoral(PlacerSubsystem placer) {
        addRequirements(placer);
        this.placer = placer;
    }


    @Override
    public void execute() {
        placer.movePlacer(-1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
