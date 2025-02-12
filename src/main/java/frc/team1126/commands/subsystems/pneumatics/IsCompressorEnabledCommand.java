package frc.team1126.commands.subsystems.pneumatics;

import frc.team1126.subsystems.PneumaticSubsystem;

public class IsCompressorEnabledCommand extends BasePneumaticsCommand {
    public IsCompressorEnabledCommand(PneumaticSubsystem subsystem) {
        super(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        subsystem.updateDashboard();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
