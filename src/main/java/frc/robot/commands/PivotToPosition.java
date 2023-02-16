package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Pivot;

public class PivotToPosition extends PIDCommand {


    public PivotToPosition(Double setpoint, Pivot pivot) {
        super(new PIDController(80, 0, 0), pivot::getMeasurement, setpoint, volts -> pivot.PIDmovePivot(volts), pivot);

    

        getController().setTolerance(0.001);
    
        //double calculatedOutput = getController().calculate(pivot.getMeasurement(), getController().getSetpoint());
    }


    @Override
  public boolean isFinished(){
    return getController().atSetpoint();
  }
  

}
