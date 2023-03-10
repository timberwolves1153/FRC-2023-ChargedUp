package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.PIDExtender;

public class ExtendIn extends CommandBase {

    PIDExtender extender;

    public ExtendIn(PIDExtender extender) {
        this.extender = extender;
        addRequirements(extender);
    }

      /** The initial subroutine of a command. Called once when the command is initially scheduled. */
    @Override
    public void initialize() {
       // extender.disable();
    }

    @Override
    public void execute() {  
        if (!extender.isExtenderInSwitchPressed()) {
            extender.setSetpoint((extender.getController().getSetpoint())-2);
        }
    }

    @Override
    public void end(boolean interrupted) {
        extender.holdPosition();
    }
}
