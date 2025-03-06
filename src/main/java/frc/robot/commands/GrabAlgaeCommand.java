package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeGrabberSubsystem;

public class GrabAlgaeCommand extends Command{
    private final AlgaeGrabberSubsystem subsystem;
    private int delayTicks;
    private AlgaeGrabProgress algaeGrabProgress;
    boolean stopped;
    private Timer spinupTimer = new Timer();

    public GrabAlgaeCommand(AlgaeGrabberSubsystem subsystem ){
        this.subsystem = subsystem;
        addRequirements(subsystem);
        delayTicks = 30;
        algaeGrabProgress = AlgaeGrabProgress.WAITING;
    }

    @Override
    public void initialize() {
        System.out.println("Initializing GrabAlgaeCommand");
        subsystem.setPositionDegrees(65);
        subsystem.setGrabberSpeed(-1);
        stopped = false;
        delayTicks = 30;
        spinupTimer.restart();
        algaeGrabProgress = AlgaeGrabProgress.WAITING;
        subsystem.setGrabberCurrentLimit(35);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("algae grabber rpm", subsystem.checkGrabberRPM());
        SmartDashboard.putNumber("algae angle", subsystem.getPositionDegrees());
        SmartDashboard.putNumber("algaeGrabProgress", algaeGrabProgress.ordinal());
        switch (algaeGrabProgress){
            case WAITING:
                
                if (spinupTimer.hasElapsed(0.4) && subsystem.checkGrabberRPM() > -30) algaeGrabProgress = AlgaeGrabProgress.SUCKING;
                break;
            
            case SUCKING:
                delayTicks--;
                if (delayTicks == 0) algaeGrabProgress = AlgaeGrabProgress.FINALISING;
                break;
            
            case FINALISING:
                subsystem.setPositionDegrees(15);
                if (subsystem.getPositionDegrees() > 14 && subsystem.getPositionDegrees() < 16) algaeGrabProgress = AlgaeGrabProgress.END;
                break;

            case END:
                break;

        }
    }

    @Override
    public boolean isFinished() {
        return algaeGrabProgress == AlgaeGrabProgress.END;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("GrabAlgaeCommand finished");
        subsystem.setGrabberCurrentLimit(10);
        if (interrupted){
            subsystem.setPositionDegrees(5);
            subsystem.setGrabberSpeed(0);
        } 
    }

    enum AlgaeGrabProgress{
        WAITING,
        SUCKING,
        FINALISING,
        END
    }
}
