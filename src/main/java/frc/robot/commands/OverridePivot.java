// package frc.robot.commands;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.PIDPivot;

// public class OverridePivot extends CommandBase{
    
//     private PIDPivot pidPivot;
//     private DoubleSupplier joystickInput;
//     private double input;

//     public OverridePivot(DoubleSupplier joystickInput, PIDPivot pidPivot) {

//         this.joystickInput = joystickInput;
//         this.pidPivot = pidPivot;
//         this.input =  input;
//         addRequirements(pidPivot);
//     }

//     @Override
//     public void execute() {
//         input = joystickInput.getAsDouble();

//         if (!pidPivot.isAtMaxHeight() && input < 0) {
//             if(Math.abs(input) > 0.1) {
//             pidPivot.incrementSetpointDegrees();
//             }

//         } else if (!pidPivot.isAtMinHeight() && input > 0) {
//             if(Math.abs(input) > 0.1) {
//             pidPivot.decrementSetpointDegrees();
//             }

//         } else {
//            pidPivot.holdPosition();
//         }
//         // }
//         // if (pidPivot.isAtMaxHeight() && joystickInput.getAsDouble() < 0) {
//         //     pidPivot.setSetpoint(pidPivot.getController().getSetpoint());
//         // } else if (pidPivot.isAtMinHeight() && joystickInput.getAsDouble() < 0) {
//         //     pidPivot.setSetpoint(pidPivot.getController().getSetpoint());
//         // } else {
//         //     pidPivot.setSetpoint((pidPivot.getController().getSetpoint()) + (0.01 * joystickInput.getAsDouble()));
//         // }
//     }

//     @Override
//     public void end(boolean interrupted) {
//         pidPivot.holdPosition();
//     }
// }
