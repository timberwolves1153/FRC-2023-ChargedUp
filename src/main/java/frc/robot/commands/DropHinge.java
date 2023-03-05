package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.PIDExtender;

public class DropHinge extends CommandBase{
    
    private PIDExtender pidExtender;
    public DropHinge(PIDExtender pidExtender) {

        this.pidExtender = pidExtender;
        addRequirements(pidExtender);
    }

    @Override
    public void initialize() {
        pidExtender.getController().setTolerance(.25);
    }

    @Override
    public void execute() {
        pidExtender.setSetpointInches(5.5);
    }

    @Override
    public void end(boolean interrupted) {
        pidExtender.holdPosition();
        System.out.println("finished");
    }

    @Override
  public boolean isFinished(){
    return pidExtender.getController().atSetpoint();
  }
  
}
