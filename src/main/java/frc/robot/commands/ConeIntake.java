package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.Collector;

public class ConeIntake extends CommandBase {

    private Collector collector;

    public ConeIntake(Collector collector) {
        this.collector = collector;
        addRequirements(collector);
    }

      /** The initial subroutine of a command. Called once when the command is initially scheduled. */
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        collector.slowConeIntake();
    }

    @Override
    public void end(boolean interrupted) {
        collector.collectorStop();
    }
}
