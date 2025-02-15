//package frc.team1126.commands.subsystems.arm;
//
//import edu.wpi.first.wpilibj2.command.Command;
//import frc.team1126.subsystems.ArmSubsystem;
//
//public class MoveArmToExtension extends Command {
//
//    private ArmSubsystem arm;
//    private double targetExtension;
//
//    public MoveArmToExtension(ArmSubsystem arm, double extension) {
//        addRequirements(arm);
//        this.arm = arm;
//        targetExtension = extension;
//    }
//
//    @Override
//    public void execute() {
//        if(arm.getExtension() > targetExtension) {
//            arm.moveExtensionToPosition(targetExtension);
//        } else if(arm.getExtension() < targetExtension) {
//            arm.moveExtensionToPosition(targetExtension);
//        }
//    }
//
//    @Override
//    public boolean isFinished() {
//
//        if(arm.getArmAngle() == targetExtension){
//            return true;
//        }
//        return false;
//    }
//
//}
