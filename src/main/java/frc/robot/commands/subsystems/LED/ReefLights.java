package frc.robot.commands.subsystems.LED;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LEDs;

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
    public void initialize() {
        if(isRight){
            if(arm.isL1) {
                ledSubsystem.setPulse(new Color8Bit(255,255,0), 1, 0, 94);
            }
            else if(arm.isL2) {
                ledSubsystem.setPulse(new Color8Bit(0,255,255), 1, 0, 94);
            }
            else if(arm.isL3) {
                ledSubsystem.setPulse(new Color8Bit(255,0,255), 1, 0, 94);
            }
            else if(arm.isL4) {
                ledSubsystem.setPulse(new Color8Bit(0,255,0), 1, 0, 94);
                
            }
        } else {
            if(arm.isL1) {
                ledSubsystem.setPulse(new Color8Bit(255,255,0), 1, 94, ledSubsystem.getLedBuffer().getLength());
            }
            else if(arm.isL2) {
                ledSubsystem.setPulse(new Color8Bit(0,255,255), 1, 94, ledSubsystem.getLedBuffer().getLength());
            }
            else if(arm.isL3) {
                ledSubsystem.setPulse(new Color8Bit(0,255,255), 1, 94, ledSubsystem.getLedBuffer().getLength());
            }
            else if(arm.isL4) {
                ledSubsystem.setPulse(new Color8Bit(0,255,0), 1, 94, ledSubsystem.getLedBuffer().getLength());
            }
        }
        ledSubsystem.update();

    }

    @Override
    public boolean isFinished() {
        return true;
    }



}
