package frc.robot.commands.pneumatics;

import frc.robot.subsystems.Pneumatics;

public class FirePiston extends BasePneumaticsCommand {
    
    public FirePiston(Pneumatics subsystem) {
        super(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.togglePistons();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.togglePistons();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
