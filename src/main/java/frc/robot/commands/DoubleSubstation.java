package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.PIDExtender;
import frc.robot.subsystems.PIDPivot;


public class DoubleSubstation extends CommandBase{
    
    private Collector collector;
    private PIDExtender extender;
    private PIDPivot pivot;

    public DoubleSubstation(Collector collector, PIDExtender extender, PIDPivot pivot) {

        this.collector = collector;
        this.extender = extender;
        this.pivot = pivot;

        addRequirements(pivot, extender, collector);
    }


    @Override
    public void execute() {
        GamePiece currPiece = collector.getCurrentGamePiece();
        if (GamePiece.CONE.equals(currPiece)) {
             pivot.setSetpointDegrees(26.5);
            extender.setSetpointInches(12.8);
        } else if (GamePiece.CUBE.equals(currPiece)) {
            pivot.setSetpointDegrees(29.5);
           extender.setSetpointInches(0.01);
        } else {
            //something has gone wrong, just dont run the collector
            pivot.holdPosition();
            extender.holdPosition();
        }
    }
}
