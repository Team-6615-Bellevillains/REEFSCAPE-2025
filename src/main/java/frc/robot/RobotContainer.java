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
import frc.robot.commands.GrabAlgaeCommand;
import frc.robot.commands.LoadCoralLaserCANCommand;
import frc.robot.commands.SpitCoralAutonCommand;
import frc.robot.commands.ThrowBallAutonCommand;
import frc.robot.commands.AlgaeAlignAssist.Target;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PivotArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.SetpointID;
import swervelib.SwerveInputStream;

@Logged
public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final SwerveSubsystem swerve = new SwerveSubsystem(driverController);
    private final AlgaeGrabberSubsystem algae = new AlgaeGrabberSubsystem();
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final PivotArmSubsystem pivot = new PivotArmSubsystem();
    @SuppressWarnings("unused") // Needs to be constructed for its periodic method to be run.
    private final LEDSubsystem ledSubsystem = new LEDSubsystem();

    private final SendableChooser<Command> autoChooser;    
    
    // from the YAGSL example project, hence the diabolical formatting
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerve.getSwerveDrive(),
                                                                () -> -driverController.getLeftY(),
                                                                () -> -driverController.getLeftX())
                                                            .withControllerRotationAxis(() -> -driverController.getRightX())
                                                            .allianceRelativeControl(true);

    SwerveInputStream driveAngularVelocitySlow = SwerveInputStream.of(swerve.getSwerveDrive(),
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

        NamedCommands.registerCommand("elevatorL1", new GoToElevatorSetpointCommand(elevator, pivot, SetpointID.L1));
        NamedCommands.registerCommand("elevatorL2", new GoToElevatorSetpointCommand(elevator, pivot, SetpointID.L2));
        NamedCommands.registerCommand("elevatorL3", new GoToElevatorSetpointCommand(elevator, pivot, SetpointID.L3));
        NamedCommands.registerCommand("elevatorL4", new GoToElevatorSetpointCommand(elevator, pivot, SetpointID.L4));
        NamedCommands.registerCommand("elevatorA1", new GoToElevatorSetpointCommand(elevator, pivot, SetpointID.A1));
        NamedCommands.registerCommand("elevatorA2", new GoToElevatorSetpointCommand(elevator, pivot, SetpointID.A2));
        NamedCommands.registerCommand("elevatorAB", new GoToElevatorSetpointCommand(elevator, pivot, SetpointID.AB));

        NamedCommands.registerCommand("loadCoralAuto", new LoadCoralLaserCANCommand(pivot));
        NamedCommands.registerCommand("spitCoralAuto", new SpitCoralAutonCommand(pivot));
        NamedCommands.registerCommand("throwBallAuto", new ThrowBallAutonCommand(pivot));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    private void configureBindings(){
        // Swerve
        swerve.setDefaultCommand(swerve.driveCommand(driveAngularVelocity, driveAngularVelocitySlow, driverController));
        driverController.a().onTrue(swerve.resetHeading());
        driverController.b().whileTrue(
            new AlgaeAlignAssist(
                swerve, 
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(), 
                false,
                Target.ALGAE
            )
        );
        
        // Algae Grabber
        driverController.x().onTrue(algae.resetAlgaeState());
        driverController.leftBumper().whileTrue(algae.spitAlgae());
        driverController.rightBumper().onTrue(new GrabAlgaeCommand(algae));

        // Elevator
        driverController.start().onTrue(elevator.zeroElevatorCommand());

        operatorController.povDown().onTrue(new GoToElevatorSetpointCommand(elevator, pivot, SetpointID.L1));
        operatorController.povLeft().onTrue(new GoToElevatorSetpointCommand(elevator, pivot, SetpointID.L2));
        operatorController.povUp().onTrue(new GoToElevatorSetpointCommand(elevator, pivot, SetpointID.L3));
        operatorController.povRight().onTrue(new GoToElevatorSetpointCommand(elevator, pivot, SetpointID.L4));
        operatorController.leftBumper().onTrue(new GoToElevatorSetpointCommand(elevator, pivot, SetpointID.A1));
        operatorController.leftTrigger().onTrue(new GoToElevatorSetpointCommand(elevator, pivot, SetpointID.A2));
        operatorController.back().onTrue(new GoToElevatorSetpointCommand(elevator, pivot, SetpointID.AB));

        // Pivot
        operatorController.a().whileTrue(pivot.spitCoral());
        operatorController.b().onTrue(pivot.invertInOut());
        operatorController.x().whileTrue(pivot.reverseCoral());
        operatorController.y().whileTrue(pivot.L1Shot());
        operatorController.rightTrigger().onTrue(pivot.grabAlgaeBargeShotCommand());
        operatorController.start().whileTrue(pivot.throwBallCommand());

        // Misc.
        operatorController.rightBumper().onTrue(new LoadCoralLaserCANCommand(pivot));
    }

    public Command getAutonomousCommand(){
        return autoChooser.getSelected();
    }
}
