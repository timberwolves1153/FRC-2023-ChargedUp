package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.PIDExtender;
import frc.robot.subsystems.PIDPivot;

public class PivotUp extends CommandBase {

    PIDPivot pivot;

    public PivotUp(PIDPivot pivot) {
        this.pivot = pivot;
        addRequirements(pivot);
    }

      /** The initial subroutine of a command. Called once when the command is initially scheduled. */
    @Override
    public void initialize() {
       // extender.disable();
    }

    @Override
    public void execute() {  
        if (!pivot.isAtMaxHeight()) {
            pivot.setSetpoint((pivot.getController().getSetpoint())+0.01);
        }
    }

    @Override
    public void end(boolean interrupted) {
        pivot.holdPosition();
    }
}
