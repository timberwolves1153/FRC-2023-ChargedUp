package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.AxisButton;
import frc.robot.Constants.GamePiece;
import frc.robot.autos.CenterScore1AndBalance;
import frc.robot.autos.LimeLightAutoAlign;
import frc.robot.autos.LimelightAlign;
import frc.robot.autos.LimelightAlignLeft;
import frc.robot.autos.LimelightAlignRight;
import frc.robot.autos.Score1AndMove;
import frc.robot.autos.Score2AndBalance;
import frc.robot.autos.Score2AndBalanceBump;
import frc.robot.autos.Score2NoBump;
import frc.robot.autos.Score2NoBumpLimelight;
import frc.robot.autos.Score2WithBump;
import frc.robot.autos.Score3NoBump;
import frc.robot.autos.ShortCenterScore1AndBalance;
import frc.robot.autos.TestBalance;
import frc.robot.commands.AutoBalanceWithRoll;
import frc.robot.commands.AutoBalanceWithRoll2;
import frc.robot.commands.CollectGamePiece;
import frc.robot.commands.DoubleSubstation;
import frc.robot.commands.ExtendIn;
import frc.robot.commands.OuttakeGamePiece;
import frc.robot.commands.PivotDown;
import frc.robot.commands.PivotUp;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.PIDExtender;
import frc.robot.subsystems.PIDPivot;
import frc.robot.subsystems.Swerve;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    //private final Joystick operator = new Joystick(1);
    private final Joystick atari = new Joystick(2);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final SlewRateLimiter filter = new SlewRateLimiter(4);

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kLeftStick.value);
    private final JoystickButton driveStart = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton driveLeftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driveRightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton driveA = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton driveB = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton driveX = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton driveY = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton driveBack = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kRightStick.value);
    private final AxisButton leftTrigger = new AxisButton(driver, XboxController.Axis.kLeftTrigger.value, .5);
    private final AxisButton rightTrigger = new AxisButton(driver, XboxController.Axis.kRightTrigger.value, .5);
    private final POVButton povDown = new POVButton(driver, 180);
    private final POVButton povUp = new POVButton(driver, 0);
    private final POVButton povLeft = new POVButton(driver, 270);
    private final POVButton povRight = new POVButton(driver, 90);

    /* Operator Buttons */

     //private final JoystickButton opYButton = new JoystickButton(operator, XboxController.Button.kY.value);
     //private final JoystickButton opAbutton = new JoystickButton(operator, XboxController.Button.kA.value);
    // private final JoystickButton opAButton = new JoystickButton(operator, XboxController.Button.kA.value);
    // private final JoystickButton opXButton = new JoystickButton(operator, XboxController.Button.kX.value);
    // private final JoystickButton opBButton = new JoystickButton(operator, XboxController.Button.kB.value);
    // private final JoystickButton opLeftBumper = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    // private final JoystickButton opRightBumper = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    // private final JoystickButton opStart = new JoystickButton(operator, XboxController.Button.kStart.value);
    // private final JoystickButton opBack = new JoystickButton(operator, XboxController.Button.kBack.value);

    private final JoystickButton atariButton1 = new JoystickButton(atari, 1);
    private final JoystickButton atariButton2 = new JoystickButton(atari, 2);
    private final JoystickButton atariButton3 = new JoystickButton(atari, 3);
    private final JoystickButton atariButton4 = new JoystickButton(atari, 4);
    private final JoystickButton atariButton5 = new JoystickButton(atari, 5);
    private final JoystickButton atariButton6 = new JoystickButton(atari, 6);
    private final JoystickButton atariButton7 = new JoystickButton(atari, 7);
    private final JoystickButton atariButton8 = new JoystickButton(atari, 8);
    private final JoystickButton atariButton9 = new JoystickButton(atari, 9);
    private final JoystickButton atariButton10 = new JoystickButton(atari, 10);
    private final JoystickButton atariButton11 = new JoystickButton(atari, 11);
    private final JoystickButton atariButton12 = new JoystickButton(atari, 12);
    private final JoystickButton atariButton13 = new JoystickButton(atari, 13);


    // private final JoystickButton opOneButton = new JoystickButton(operator, 1);
    // private final JoystickButton opTwoButton = new JoystickButton(operator, 2);
    // private final JoystickButton opThreeButton = new JoystickButton(operator, 3);



    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final PIDPivot pidPivot = new PIDPivot();
    private final PIDExtender pidExtender = new PIDExtender();
    private final Collector collector = new Collector();
    private final LEDLights lights = new LEDLights();
    private final LimelightAlign limelightAlign = new LimelightAlign(s_Swerve);
    private final LimelightAlignLeft limelightAlignLeft = new LimelightAlignLeft(s_Swerve);
    private final LimelightAlignRight limelightAlignRight = new LimelightAlignRight(s_Swerve);
    private final LimeLightAutoAlign limeLightAutoAlign = new LimeLightAutoAlign(s_Swerve);


    private final Score2NoBump score2NoBump = new Score2NoBump(s_Swerve, collector, pidExtender, pidPivot);
    private final Score2WithBump score2WithBump = new Score2WithBump(s_Swerve, collector, pidExtender, pidPivot);
    private final Score1AndMove score1AndMove = new Score1AndMove(s_Swerve, collector, pidExtender, pidPivot);
    private final CenterScore1AndBalance centerScore1AndBalance = new CenterScore1AndBalance(s_Swerve, collector, pidExtender, pidPivot);
    private final TestBalance testBalance = new TestBalance(s_Swerve);
    private final Score2NoBumpLimelight score2NoBumpLimeLight = new Score2NoBumpLimelight(s_Swerve, collector, pidExtender, pidPivot, limeLightAutoAlign, limelightAlign);
    private final Score2AndBalance score2AndBalance = new Score2AndBalance(s_Swerve, collector, pidExtender, pidPivot);
    private final Score3NoBump score3NoBump = new Score3NoBump(s_Swerve, collector, pidExtender, pidPivot);
    private final Score2AndBalanceBump score2AndBalanceBump = new Score2AndBalanceBump(s_Swerve, collector, pidExtender, pidPivot);
    private final ShortCenterScore1AndBalance shortCenterScore1AndBalance = new ShortCenterScore1AndBalance(s_Swerve, collector, pidExtender, pidPivot);


    private final AutoBalanceWithRoll autoBalanceWithRoll; 
    private final AutoBalanceWithRoll2 autoBalanceWithRoll2;
    private SendableChooser<Command> autoCommandChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, 
        () -> adjustedRobotTranslationSpeed(), 
        () -> adjustedRobotStrafeSpeed(), 
        () -> adjustedRobotRotationSpeed(), 
         robotCentric,
         driveRightBumper,
         driveY,
         driveB,
         driveA,
         driveX, 
         pidPivot));
        
        
         
        autoBalanceWithRoll = new AutoBalanceWithRoll(s_Swerve, () -> robotCentric.getAsBoolean()); 
        autoBalanceWithRoll2 = new AutoBalanceWithRoll2(s_Swerve, () -> robotCentric.getAsBoolean());
        
        autoCommandChooser = new SendableChooser<Command>();
        autoCommandChooser.setDefaultOption("Score 1 And Move", score1AndMove);
        autoCommandChooser.addOption("Score 1 And Move", score1AndMove);
        autoCommandChooser.addOption("Score 1 and Balance Center", centerScore1AndBalance);
        autoCommandChooser.addOption("Score 2 No Bump", score2NoBump);
        autoCommandChooser.addOption("Score 2 And Balance", score2AndBalance);
        autoCommandChooser.addOption("Score 2 Bump", score2WithBump);
        autoCommandChooser.addOption("Score 2 And Balance Bump", score2AndBalanceBump);
        autoCommandChooser.addOption("Score 3 No Bump", score3NoBump);
        autoCommandChooser.addOption("Short Center Balance", shortCenterScore1AndBalance);
        //autoCommandChooser.addOption("Test Balance", testBalance);

        SmartDashboard.putData("Auto Command Chooser", autoCommandChooser);
        

      
        // Configure the button bindings
        configureButtonBindings();
        lights.setRGB(125, 249, 255);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        
        /* Driver Buttons */
        //zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        // driveA.whileTrue(autoBalanceWithRoll);
        // driveA.onFalse(new InstantCommand(() -> s_Swerve.stop())); 
        // driveX.whileTrue(new InstantCommand(() -> s_Swerve.xPosition(true)));
        robotCentric.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
        // driveBack.onTrue(new InstantCommand(() -> collector.toggleHinge()));
        // driveA.onTrue(Commands.runOnce(() -> pidExtender.setSetpointInches(-2), pidExtender));
        driveLeftBumper.onTrue(new InstantCommand(() -> limelightAlign.generateAlignCommand().schedule()));
        driveLeftBumper.onFalse(new InstantCommand(() -> {if(limelightAlign.getCommand() != null) limelightAlign.getCommand().cancel();}));
        
        leftTrigger.onTrue(new InstantCommand(() -> limelightAlignLeft.generateAlignCommand().schedule()));
        leftTrigger.onFalse(new InstantCommand(() -> {if(limelightAlignLeft.getCommand() != null) limelightAlignLeft.getCommand().cancel();}));
        
        rightTrigger.onTrue(new InstantCommand(() -> limelightAlignRight.generateAlignCommand().schedule()));
        rightTrigger.onFalse(new InstantCommand(() -> {if(limelightAlignRight.getCommand() != null) limelightAlignRight.getCommand().cancel();}));
        
        povDown.onTrue(new InstantCommand(()-> s_Swerve.zeroGyro()));
        
        driveBack.whileTrue(new InstantCommand(() -> s_Swerve.xPosition(true)));
        povUp.whileTrue(autoBalanceWithRoll);
        povDown.whileTrue(autoBalanceWithRoll2);

        

        
        //driveB.onTrue(new InstantCommand(() -> ledLights.setRGB(218, 165, 0)));
        //driveY.onTrue(new InstantCommand(() -> ledLights.setRGB(128, 0, 128)));
        // driveX.onTrue(new InstantCommand(() -> ledLights.setRGB(0, 255, 0)));
        //opAbutton.onTrue(goToTunableDegree);
        //opYButton.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(-2), pidPivot));
        /* Operator Buttons */

        // if (Constants.competitionRobot) {

            //opYButton.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(0), pidPivot));
            atariButton1.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(-35), pidPivot));
            atariButton1.onTrue(Commands.runOnce(() -> pidExtender.setSetpointInches(-1), pidExtender));
            atariButton2.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(22), pidPivot));
            atariButton2.onTrue(Commands.runOnce(() -> pidExtender.setSetpointInches(9), pidExtender));
            atariButton3.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(33), pidPivot));
            atariButton3.onTrue(Commands.runOnce(() -> pidExtender.setSetpointInches(27.5), pidExtender));

            // atariButton4.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(23.5), pidPivot));
            // atariButton4.onTrue(Commands.runOnce(() -> pidExtender.setSetpointInches(0.01), pidExtender));
            atariButton4.onTrue(new DoubleSubstation(collector, pidExtender, pidPivot));

            //atariButton5.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(-45), pidPivot));
           // atariButton5.onTrue(Commands.runOnce(() -> pidExtender.setSetpointInches(15), pidExtender));

            atariButton6.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(-50), pidPivot));
            atariButton6.onTrue(Commands.runOnce(() -> pidExtender.setSetpointInches(0.5), pidExtender));

            atariButton7.whileTrue(new CollectGamePiece(collector));
            atariButton8.whileTrue(new OuttakeGamePiece(collector));
            atariButton13.onTrue(new InstantCommand(() -> collector.setCurrentGamePiece(GamePiece.CUBE)));
            atariButton13.onTrue(new InstantCommand(() -> lights.setPurple()));
            atariButton13.onFalse(new InstantCommand(() -> collector.setCurrentGamePiece(GamePiece.CONE)));
            atariButton13.onFalse(new InstantCommand(() -> lights.setYellow()));

           
            // we need to set an initial game piece
            if (atariButton13.getAsBoolean()) {
                collector.setCurrentGamePiece(GamePiece.CUBE);
                
            } else {
                collector.setCurrentGamePiece(GamePiece.CONE);
            }

            atariButton9.whileTrue(Commands.runOnce(() -> {pidExtender.disable(); pidExtender.extendOut();}));
            atariButton9.whileTrue(new InstantCommand(() -> pidExtender.extendOut()));
            atariButton9.onFalse(new InstantCommand(() -> pidExtender.holdPosition()));

            atariButton10.whileTrue(new ExtendIn(pidExtender));
            atariButton10.onFalse(new InstantCommand(() -> pidExtender.holdPosition()));

            atariButton11.whileTrue(new PivotUp(pidPivot));
            atariButton11.onFalse(new InstantCommand(() -> pidPivot.holdPosition()));

            atariButton12.whileTrue(new PivotDown(pidPivot));
            atariButton12.onFalse(new InstantCommand(() -> pidPivot.holdPosition()));

        // } else {
    //         opYButton.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(0), pidPivot));
    //     atariButton1.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(-50), pidPivot));
    //     atariButton2.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(15), pidPivot));
    //     //atariButton2.onTrue(Commands.runOnce(() -> pidExtender.setSetpointInches(7), pidExtender));
    //     atariButton3.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(19), pidPivot));
    //     //atariButton3.onTrue(Commands.runOnce(() -> pidExtender.setSetpointInches(22.9), pidExtender));
        
    //     atariButton5.onFalse(new InstantCommand(() -> pidExtender.holdPosition()));
    //     atariButton5.onTrue(Commands.runOnce(() -> {pidExtender.disable(); pidExtender.extendOut();}));

    //     atariButton6.whileTrue(new ExtendIn(pidExtender));
    //     atariButton6.onFalse(new InstantCommand(() -> pidExtender.holdPosition()));

    //     // atariButton7.whileTrue(new InstantCommand(() -> collector.coneIntake()));
    //     // atariButton7.onFalse(new InstantCommand(() -> collector.slowIntake()));

    //     // atariButton8.whileTrue(new InstantCommand(() -> collector.coneOuttake()));
    //     // atariButton8.onFalse(new InstantCommand(() -> collector.slowOuttake()));

    //     atariButton10.onTrue(Commands.runOnce(() -> pidPivot.setSetpointDegrees(29), pidPivot));

    //    // atariButton12.onTrue(Commands.runOnce(() -> pidExtender.setSetpointInches(-5), pidExtender));
    //     //atariButton12.onTrue(Commands.runOnce(() -> pidPivot.setSetpoint(0.5), pidPivot));
    //    // }

    //    atariButton7.whileTrue(new PivotUp(pidPivot));
    //     atariButton7.onFalse(new InstantCommand(() -> pidPivot.holdPosition()));

    //    atariButton8.whileTrue(new PivotDown(pidPivot));
    //    atariButton8.onFalse(new InstantCommand(() -> pidPivot.holdPosition()));
    }

    public void resetToAbsolute() {
        s_Swerve.resetModulesToAbsolute();
    }

    // public void lockHinge() {
    //     collector.lockHinge();
    // }

    // public void unlockHinge() {
    //     collector.openHinge();
    // }

    public Double adjustedRobotTranslationSpeed() {
        double defaultSpeed = filter.calculate(-driver.getRawAxis(translationAxis));
        double multiplier = 1;
        double currentAngle = pidPivot.getDegrees();
        return defaultSpeed * multiplier;
    }

    public Double adjustedRobotStrafeSpeed() {
        double defaultSpeed = -driver.getRawAxis(strafeAxis);
        double multiplier = 1;
        double currentAngle = pidPivot.getDegrees();
        return defaultSpeed * multiplier;
    }

    public Double adjustedRobotRotationSpeed() {
        double defaultSpeed = -driver.getRawAxis(rotationAxis) * 0.85;
        double multiplier = 1;
        double currentAngle = pidPivot.getDegrees();
        return defaultSpeed * multiplier;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //return new TestAuto(testTrajectory, , s_Swerve);
        //return null;
       //return autoCommandChooser.getSelected();
      // return new ScoreAndMove(s_Swerve, pivot, collector);
      return autoCommandChooser.getSelected();//new ScoreAndBalanceTest(s_Swerve);
    }

    public void setInitGamePeiceLights() {
        if (atariButton13.getAsBoolean()) {
            lights.setPurple();
        } else {
            lights.setYellow();
        }
    }

}
