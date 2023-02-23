// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.PIDCommand;
// import frc.robot.subsystems.Extender;

// public class ExtendToPosition extends PIDCommand{
    

//     public ExtendToPosition(double setpoint, Extender extender) {
//         super(extender.getController(), extender::getMeasurement, setpoint, volts -> extender.extenderPidMove(volts), extender);
//     }

//     @Override
//     public boolean isFinished() {
//         return getController().atSetpoint();
//     }
// }
