package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PIDPivot;

public class OverridePivot extends CommandBase{
    
    private PIDPivot pidPivot;
    private DoubleSupplier joystickInput;
    private double input;

    public OverridePivot(DoubleSupplier joystickInput, PIDPivot pidPivot) {

        this.joystickInput = joystickInput;
        this.pidPivot = pidPivot;
        addRequirements(pidPivot);
    }

    @Override
    public void execute() {
        input = joystickInput.getAsDouble();
        if(Math.abs(input) > 0.1) {
            pidPivot.disable();
            double volts = input * 12;
            double clampedOutput = MathUtil.clamp(volts, -4.7, 3);
            pidPivot.teleopMovePivot(clampedOutput);
        } else {
            
            pidPivot.enable();
            
        }
    }
}
