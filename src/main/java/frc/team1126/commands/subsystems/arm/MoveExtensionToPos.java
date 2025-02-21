package frc.team1126.commands.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.ArmSubsystem;
import frc.team1126.subsystems.ExtensionSubsystem;

public class MoveExtensionToPos extends Command {

   private ExtensionSubsystem extension;
   private ArmSubsystem arm;
   private double targetExtension;

   public MoveExtensionToPos(ExtensionSubsystem extension, ArmSubsystem arm, double pos) {
       addRequirements(RobotContainer.m_extension);
       this.extension = extension;
       this.arm = arm;
       targetExtension = pos;
   }

   @Override
   public void execute() {
    if(arm.getArmAngle() > 30) {
        extension.extReachGoal(targetExtension);
    }
       
    
   }

   @Override
   public boolean isFinished() {
       return false;
   }

}
