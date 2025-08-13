package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.SetpointID;

public class GoToElevatorSetpointCommand extends Command{
    ElevatorSubsystem elevator;
    PivotArmSubsystem pivot;
    ElevatorSubsystem.SetpointID setpointID;

    public GoToElevatorSetpointCommand(ElevatorSubsystem elevatorSubsystem, PivotArmSubsystem pivotArmSubsystem, ElevatorSubsystem.SetpointID setpointID){
        this.addRequirements(elevatorSubsystem, pivotArmSubsystem);

        this.elevator = elevatorSubsystem;
        this.pivot = pivotArmSubsystem;
        this.setpointID = setpointID;
    }
    
    @Override
    public void initialize() {
        if(elevator.getCurrentSetpointID() == setpointID){
            return;
        }

        pivot.setArmPosition(setpointID == SetpointID.AB ? 0 : 2);
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("GoToElevatorPositionCommand.isFinished()", elevator.atPosition());
        return elevator.atPosition();
    }

    @Override
    public void end(boolean interrupted) {
        switch (setpointID) {
            case L1:
            case AB:
                pivot.setArmPosition(0);
                break;
            case A1:
            case A2:
                pivot.setArmPosition(2);
                pivot.setGrabberMotor(0.2);
                break;
            default:
                pivot.setArmPosition(2);
                break;
        }
    }
}
