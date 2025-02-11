package frc.robot.commands.pneumatics;

import frc.robot.subsystems.Pneumatics;

public class PowerCompressorCommand extends BasePneumaticsCommand {
    public PowerCompressorCommand(Pneumatics subsystem) {
        super(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        subsystem.disableCompressor();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subsystem.enableCompressor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
