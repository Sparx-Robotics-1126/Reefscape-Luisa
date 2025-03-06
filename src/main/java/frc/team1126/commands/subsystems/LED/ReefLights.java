package frc.team1126.commands.subsystems.LED;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.ArmSubsystem;
import frc.team1126.subsystems.LEDs;

public class ReefLights extends Command {
    LEDs ledSubsystem;
    ArmSubsystem arm;
    boolean isRight;

    public ReefLights(LEDs ledSubsystem, ArmSubsystem arm, boolean isRight){
        addRequirements(RobotContainer.ledSubsystem);
        this.ledSubsystem = ledSubsystem;
        this.arm = arm;
        this.isRight = isRight;

    }
    
    // halfway is 94

    @Override
    public void execute() {
        if(isRight){
            if(arm.isL1){
                ledSubsystem.setPulse(new Color8Bit(255,255,0), 1, 0);
            }
            if(arm.isL2){
                ledSubsystem.setPulse(new Color8Bit(0,255,255), 1, 0);
            }
            if(arm.isL3){
                ledSubsystem.setPulse(new Color8Bit(255,0,255), 1, 0);
            }
            if(arm.isL4){
                ledSubsystem.setPulse(new Color8Bit(0,255,0), 1, 0);
                
            }
        } else {
            if(arm.isL1){
                ledSubsystem.setPulse(new Color8Bit(255,255,0), 1, 94);
            }
            if(arm.isL2){
                ledSubsystem.setPulse(new Color8Bit(0,255,255), 1, 94);
            }
            if(arm.isL3){
                ledSubsystem.setPulse(new Color8Bit(0,255,255), 1, 94);
            }
            if(arm.isL4){
                ledSubsystem.setPulse(new Color8Bit(0,255,0), 1, 94);
            }
        }

    }



}
