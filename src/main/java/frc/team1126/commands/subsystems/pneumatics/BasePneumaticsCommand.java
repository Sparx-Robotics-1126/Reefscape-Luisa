package frc.team1126.commands.subsystems.pneumatics;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.subsystems.PneumaticSubsystem;

public abstract class BasePneumaticsCommand extends Command {
    protected PneumaticSubsystem subsystem;

    protected BasePneumaticsCommand(PneumaticSubsystem subsystem) {
        addRequirements(subsystem);
        this.subsystem = subsystem;
    }
}
