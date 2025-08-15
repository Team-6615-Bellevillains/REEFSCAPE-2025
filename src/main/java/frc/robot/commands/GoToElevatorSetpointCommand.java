package frc.robot.commands;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.SetpointID;
import frc.robot.subsystems.PivotArmSubsystem.Position;

public class GoToElevatorSetpointCommand extends Command{
    private final ElevatorSubsystem elevator;
    private final PivotArmSubsystem pivot;
    private final ElevatorSubsystem.SetpointID setpointID;

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

        elevator.setReference(setpointID);
        pivot.setArmPosition(setpointID == SetpointID.AB ? Position.IN : Position.OUT);
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
                pivot.setArmPosition(Position.IN);
                break;
            case A1:
            case A2:
                pivot.setArmPosition(Position.OUT);
                pivot.setGrabberPower(Percent.of(20));
                break;
            default:
                pivot.setArmPosition(Position.OUT);
                break;
        }
    }
}
