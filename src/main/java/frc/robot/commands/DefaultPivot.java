package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;

public class DefaultPivot extends CommandBase {
    
    private Pivot pivot;
    private DoubleSupplier volts;

    public DefaultPivot(DoubleSupplier volts, Pivot pivot) {

        this.volts = volts;
        this.pivot = pivot;
        addRequirements(pivot);
    }

    @Override
    public void execute() {
        pivot.teleopMovePivot(volts.getAsDouble() * 12);
    }
}
