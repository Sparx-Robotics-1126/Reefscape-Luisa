package frc.team1126.commands.subsystems.arm;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1126.RobotContainer;
import frc.team1126.subsystems.ArmSubsystem;
import frc.team1126.subsystems.ExtensionSubsystem;

public class MoveExtHome extends Command {

   private ExtensionSubsystem extension;
   private ArmSubsystem arm;
   private double targetExtension;
   private ClosedLoopSlot slot;

   public MoveExtHome(ExtensionSubsystem extension, ArmSubsystem arm, double pos, ClosedLoopSlot slot) {
       addRequirements(RobotContainer.m_extension);
       this.extension = extension;
       this.arm = arm;
       targetExtension = pos;
       this.slot = slot;
   }

   @Override
   public void execute() {
        extension.extReachGoal(targetExtension, slot);
       
    
   }

   @Override
   public boolean isFinished() {
       return false;
   }

}
