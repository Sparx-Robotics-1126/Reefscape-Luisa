package frc.team1126.commands.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.ExtensionSubsystem;

public class MoveExtHome extends Command {

   private ExtensionSubsystem extension;
   private double targetExtension;

   public MoveExtHome(ExtensionSubsystem extension, double pos) {
       addRequirements(RobotContainer.m_extension);
       this.extension = extension;
       targetExtension = pos;
   }

   @Override
   public void execute() {
        extension.extReachGoal(targetExtension);
       
    
   }

   @Override
   public boolean isFinished() {
       return false;
   }

}
