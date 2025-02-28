package frc.team1126.commands.subsystems.LED;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.subsystems.LEDs;

public class PulseCommand extends Command {
    private final LEDs ledSubsystem;
    private final Color8Bit color;
    private final int pulseRate;
    private int index;

    public PulseCommand(LEDs ledSubsystem, Color8Bit color, int pulseRate, int startingIndex) {
        this.ledSubsystem = ledSubsystem;
        this.color = color;
        this.pulseRate = pulseRate;
        index = startingIndex;
        addRequirements(ledSubsystem);

        // halfway is 94
    }

    @Override
    public void execute() {
        ledSubsystem.setPulse(color, pulseRate, index);
    }

    @Override
    public boolean isFinished() {
        return false; // Run until interrupted
    }
}
