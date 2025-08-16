package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.AlgaeAlignAssist;
import frc.robot.commands.GoToElevatorSetpointCommand;
import frc.robot.commands.IntakeFloorAlgaeCommand;
import frc.robot.commands.LoadCoralLaserCANCommand;
import frc.robot.commands.SpitCoralAutonCommand;
import frc.robot.commands.ThrowBallAutonCommand;
import frc.robot.commands.AlgaeAlignAssist.Target;
import frc.robot.subsystems.FloorAlgaeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PivotSubsytem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.SetpointID;
import swervelib.SwerveInputStream;

@Logged
public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final FloorAlgaeSubsystem floorAlgaeSubsystem = new FloorAlgaeSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final PivotSubsytem pivotSubsystem = new PivotSubsytem();
    @SuppressWarnings("unused") // Needs to be constructed for its periodic method to be run.
    private final LEDSubsystem ledSubsystem = new LEDSubsystem();

    private final SendableChooser<Command> autoChooser;    
    
    // from the YAGSL example project, hence the diabolical formatting
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
                                                                () -> -driverController.getLeftY(),
                                                                () -> -driverController.getLeftX())
                                                            .withControllerRotationAxis(() -> -driverController.getRightX())
                                                            .allianceRelativeControl(true);

    SwerveInputStream driveAngularVelocitySlow = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
                                                                () -> driverController.getLeftY() * -0.5,
                                                                () -> driverController.getLeftX() * -0.5)
                                                            .withControllerRotationAxis(driverController::getRightX)
                                                            .deadband(0.05)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true)
                                                            .cubeRotationControllerAxis(true)
                                                            .cubeTranslationControllerAxis(true);


    public RobotContainer(){
        // Robot is always loaded at start of Autonomous
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> SharedState.get().setLoaded(true)));

        NamedCommands.registerCommand("elevatorL1", new GoToElevatorSetpointCommand(elevatorSubsystem, pivotSubsystem, SetpointID.L1));
        NamedCommands.registerCommand("elevatorL2", new GoToElevatorSetpointCommand(elevatorSubsystem, pivotSubsystem, SetpointID.L2));
        NamedCommands.registerCommand("elevatorL3", new GoToElevatorSetpointCommand(elevatorSubsystem, pivotSubsystem, SetpointID.L3));
        NamedCommands.registerCommand("elevatorL4", new GoToElevatorSetpointCommand(elevatorSubsystem, pivotSubsystem, SetpointID.L4));
        NamedCommands.registerCommand("elevatorA1", new GoToElevatorSetpointCommand(elevatorSubsystem, pivotSubsystem, SetpointID.A1));
        NamedCommands.registerCommand("elevatorA2", new GoToElevatorSetpointCommand(elevatorSubsystem, pivotSubsystem, SetpointID.A2));
        NamedCommands.registerCommand("elevatorAB", new GoToElevatorSetpointCommand(elevatorSubsystem, pivotSubsystem, SetpointID.AB));

        NamedCommands.registerCommand("loadCoralAuto", new LoadCoralLaserCANCommand(pivotSubsystem));
        NamedCommands.registerCommand("spitCoralAuto", new SpitCoralAutonCommand(pivotSubsystem));
        NamedCommands.registerCommand("throwBallAuto", new ThrowBallAutonCommand(pivotSubsystem));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    private void configureBindings(){
        // Swerve
        swerveSubsystem.setDefaultCommand(swerveSubsystem.driveCommand(driveAngularVelocity, driveAngularVelocitySlow, driverController));
        driverController.a().onTrue(swerveSubsystem.resetHeading());
        driverController.b().whileTrue(
            new AlgaeAlignAssist(
                swerveSubsystem, 
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(), 
                false,
                Target.ALGAE
            )
        );
        
        // Algae Grabber
        driverController.x().onTrue(floorAlgaeSubsystem.resetAlgaeState());
        driverController.leftBumper().whileTrue(floorAlgaeSubsystem.spitAlgae());
        driverController.rightBumper().onTrue(new IntakeFloorAlgaeCommand(floorAlgaeSubsystem));

        // Elevator
        driverController.start().onTrue(elevatorSubsystem.zeroElevatorCommand());

        operatorController.povDown().onTrue(new GoToElevatorSetpointCommand(elevatorSubsystem, pivotSubsystem, SetpointID.L1));
        operatorController.povLeft().onTrue(new GoToElevatorSetpointCommand(elevatorSubsystem, pivotSubsystem, SetpointID.L2));
        operatorController.povUp().onTrue(new GoToElevatorSetpointCommand(elevatorSubsystem, pivotSubsystem, SetpointID.L3));
        operatorController.povRight().onTrue(new GoToElevatorSetpointCommand(elevatorSubsystem, pivotSubsystem, SetpointID.L4));
        operatorController.leftBumper().onTrue(new GoToElevatorSetpointCommand(elevatorSubsystem, pivotSubsystem, SetpointID.A1));
        operatorController.leftTrigger().onTrue(new GoToElevatorSetpointCommand(elevatorSubsystem, pivotSubsystem, SetpointID.A2));
        operatorController.back().onTrue(new GoToElevatorSetpointCommand(elevatorSubsystem, pivotSubsystem, SetpointID.AB));

        // Pivot
        operatorController.a().whileTrue(pivotSubsystem.spitCoral());
        operatorController.b().onTrue(pivotSubsystem.invertInOut());
        operatorController.x().whileTrue(pivotSubsystem.reverseCoral());
        operatorController.y().whileTrue(pivotSubsystem.L1Shot());
        operatorController.rightTrigger().onTrue(pivotSubsystem.grabAlgaeBargeShotCommand());
        operatorController.start().whileTrue(pivotSubsystem.throwBallCommand());

        // Misc.
        operatorController.rightBumper().onTrue(new LoadCoralLaserCANCommand(pivotSubsystem));
    }

    public Command getAutonomousCommand(){
        return autoChooser.getSelected();
    }
}
