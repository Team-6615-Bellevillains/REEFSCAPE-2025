package frc.robot.commands;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsytem;
import frc.robot.subsystems.ElevatorSubsystem.SetpointID;
import frc.robot.subsystems.PivotSubsytem.Position;

public class GoToElevatorSetpointCommand extends Command{
    private final ElevatorSubsystem elevatorSubsystem;
    private final PivotSubsytem pivotSubsystem;
    private final ElevatorSubsystem.SetpointID setpointID;

    public GoToElevatorSetpointCommand(ElevatorSubsystem elevatorSubsystem, PivotSubsytem pivotSubsystem, ElevatorSubsystem.SetpointID setpointID){
        this.addRequirements(elevatorSubsystem, pivotSubsystem);

        this.elevatorSubsystem = elevatorSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.setpointID = setpointID;
    }
    
    @Override
    public void initialize() {
        if(elevatorSubsystem.getCurrentSetpointID() == setpointID){
            return;
        }

        elevatorSubsystem.setReference(setpointID);
        pivotSubsystem.setArmPosition(setpointID == SetpointID.AB ? Position.IN : Position.OUT);
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("GoToElevatorPositionCommand.isFinished()", elevatorSubsystem.atPosition());
        return elevatorSubsystem.atPosition();
    }

    @Override
    public void end(boolean interrupted) {
        switch (setpointID) {
            case L1:
            case AB:
                pivotSubsystem.setArmPosition(Position.IN);
                break;
            case A1:
            case A2:
                pivotSubsystem.setArmPosition(Position.OUT);
                pivotSubsystem.setGrabberPower(Percent.of(20));
                break;
            default:
                pivotSubsystem.setArmPosition(Position.OUT);
                break;
        }
    }
}
