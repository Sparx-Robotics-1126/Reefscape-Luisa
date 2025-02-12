package frc.team1126.commands.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.subsystems.ArmSubsystem;

public class MoveArmToAngle extends Command {
    
    private ArmSubsystem arm;
    private double targetAngle;

    public MoveArmToAngle(ArmSubsystem arm, double angle) {
        addRequirements(arm);
        this.arm = arm;
        targetAngle = angle;
    }

    @Override
    public void execute() {
        if(arm.getArmAngle() > targetAngle) {
            arm.moveToAngle(targetAngle);
        } else if(arm.getArmAngle() < targetAngle) {
            arm.moveToAngle(targetAngle);
        }
    }

    @Override
    public boolean isFinished() {
        
        if(arm.getArmAngle() == targetAngle){
            return true;
        }
        return false;
    }
}
