package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.PIDExtender;
import frc.robot.subsystems.PIDPivot;

public class PivotDown extends CommandBase {

    PIDPivot pivot;

    public PivotDown(PIDPivot pivot) {
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
        if (!pivot.isAtMinHeight()) {
            pivot.setSetpoint((pivot.getController().getSetpoint())-0.01);
        }
    }

    @Override
    public void end(boolean interrupted) {
        pivot.holdPosition();
    }
}
