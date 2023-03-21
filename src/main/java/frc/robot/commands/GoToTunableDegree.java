package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PIDPivot;

public class GoToTunableDegree extends CommandBase{

    private PIDPivot pivot;
    private double pivotSetpoint;

    public GoToTunableDegree( PIDPivot pivot) {

        this.pivot = pivot;

    }

    @Override
    public void initialize() {
        pivotSetpoint = SmartDashboard.getNumber("Set Pivot Setpoint Degrees", 0);
    }

    @Override
    public void execute() {
        pivot.setSetpointDegrees(pivotSetpoint);
    }
}
