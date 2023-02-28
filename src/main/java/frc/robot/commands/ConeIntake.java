package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;

public class ConeIntake extends CommandBase {
    
    private Collector collector;
    public ConeIntake(Collector collector) {
        this.collector = collector;
        addRequirements(collector);
    }

    @Override
    public void execute() {
        collector.coneNormalIntake();
    }

    @Override
    public void end(boolean interrupted) {
        collector.slowConeIntake();
    }
}
