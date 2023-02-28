package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;

public class CubeIntake extends CommandBase{
    
    private Collector collector;
    public CubeIntake(Collector collector) {

        this.collector = collector;
        addRequirements(collector);
    }

    @Override
    public void execute() {
        collector.cubeNormalIntake();
    }

    @Override
    public void end(boolean interrupted) {
        collector.slowCubeIntake();
    }
}
