package frc.team1126.commands.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.ArmSubsystem;
import frc.team1126.subsystems.ExtensionSubsystem;

public class MoveExtensionToPos extends Command {

   private ExtensionSubsystem extension;
   private double targetExtension;

   public MoveExtensionToPos(ExtensionSubsystem extension, double pos) {
       addRequirements(RobotContainer.m_extension);
       this.extension = extension;
       targetExtension = pos;
   }

   @Override
   public void execute() {
    //    if(arm.getExtension() > targetExtension) {
    //        arm.moveExtensionToPosition(targetExtension);
    //    } else if(arm.getExtension() < targetExtension) {
    //        arm.moveExtensionToPosition(targetExtension);
    //    }
    extension.extReachGoal(targetExtension);
    
   }

   @Override
   public boolean isFinished() {

    //    if(extension.getExtension() > targetExtension){
    //        return true;
    //    }
       return false;
   }

}
