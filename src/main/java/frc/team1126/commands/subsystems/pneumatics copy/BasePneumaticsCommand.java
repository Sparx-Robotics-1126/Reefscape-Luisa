package frc.robot.commands.pneumatics;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pneumatics;

public abstract class BasePneumaticsCommand extends Command {
    protected Pneumatics subsystem;

    protected BasePneumaticsCommand(Pneumatics subsystem) {
        addRequirements(subsystem);
        this.subsystem = subsystem;
    }
}
