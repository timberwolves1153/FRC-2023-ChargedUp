package frc.robot.autos;

import frc.robot.commands.AutoBalanceWithRoll;
import frc.robot.subsystems.Swerve;

public class TestBalance extends PPAutoBase{
    
    private Swerve swerve;
    public TestBalance(Swerve swerve) {
        super(swerve);
        addRequirements(swerve);


        addCommands(
            new AutoBalanceWithRoll(swerve, () -> true)
        );
    }
}
