package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;

public class DefaultPivot extends CommandBase {
    
    private Pivot pivot;
    private DoubleSupplier joystickInput;
    private double input;

    public DefaultPivot(DoubleSupplier joystickInput, Pivot pivot) {

        this.joystickInput = joystickInput;
        this.pivot = pivot;
        addRequirements(pivot);
         
    }

    @Override
    public void execute() {

        // if (pivot.isAtMaxHeight() || pivot.isAtMinHeight()) {
        //     pivot.teleopMovePivot(0);
        // } else {
        //      if (Math.abs(input) > .1) {
        //         pivot.teleopMovePivot(joystickInput.getAsDouble() * 12);
        //     }
        // }
        
        input = joystickInput.getAsDouble();
        // if (input > 0 && pivot.isAtMaxHeight()) {
        //     pivot.teleopMovePivot(0);
        // } else if (input < 0 && pivot.isAtMinHeight()) {
        //     pivot.teleopMovePivot(0);
        // } else {
        //     if (Math.abs(input) > .1) {
          //  pivot.teleopMovePivot(joystickInput.getAsDouble() * 12);
      //  }
        // }

        SmartDashboard.putNumber("joystick input", input);
        
        if (Math.abs(input) > .1) {
            double volts = input * 12;
            double clampedOutput = MathUtil.clamp(volts, -7, 6.75);
            pivot.teleopMovePivot(clampedOutput);
            SmartDashboard.putNumber("joystick clamped", clampedOutput);
        } else {
            pivot.teleopMovePivot(0);
        }
    }
}

