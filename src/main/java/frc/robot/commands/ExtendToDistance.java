// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Extender;

// public class ExtendToDistance extends CommandBase{
//     private Extender extender;

//     public ExtendToDistance(Extender extender){
//         this.extender = extender;
//         addRequirements(extender);
//     }

//     @Override
//     public void initialize(){}

//     @Override
//     public void execute(){
//         if(extender.getExtenderSwitch()){
//             extender.set(0);
//             extender.reset();
//         }
//         else{
//             extender.set(-0.3);
//         }
//     }
//     // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     extender.set(0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
