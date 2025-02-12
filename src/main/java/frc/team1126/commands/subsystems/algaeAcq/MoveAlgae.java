package frc.team1126.commands.subsystems.algaeAcq;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.AlgaeAcquisition;

public class MoveAlgae extends Command {

    private DoubleSupplier m_power;
    private AlgaeAcquisition m_algae;

    public MoveAlgae(AlgaeAcquisition algae, DoubleSupplier power) {
        addRequirements(RobotContainer.m_algae);
        m_algae = algae;
        m_power = power;
    }

    @Override
    public void execute() {
        double speed = MathUtil.applyDeadband(m_power.getAsDouble(), .1);
        m_algae.moveArm(speed);
    }
    
}
