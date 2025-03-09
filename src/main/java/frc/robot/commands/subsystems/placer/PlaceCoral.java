package frc.robot.commands.subsystems.placer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PlacerSubsystem;

public class PlaceCoral extends Command {
    
    private PlacerSubsystem placer;
    
    public PlaceCoral(PlacerSubsystem placer) {
        addRequirements(placer);
        this.placer = placer;
    }
    
    @Override
    public void execute() {
        placer.movePlacer(.5);
    }

    @Override
    public void end(boolean interruped){
        placer.movePlacer(0);
    }
}
