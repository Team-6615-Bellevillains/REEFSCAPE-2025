package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeGrabberSubsystem extends SubsystemBase {
    private final SparkFlex angleMotor = new SparkFlex(30, MotorType.kBrushless);
    private final SparkClosedLoopController angleController = angleMotor.getClosedLoopController();
    private final SparkFlex grabberMotor = new SparkFlex(31, MotorType.kBrushless);
    public static final double CONVERSION_FACTOR = 20/360;

    public AlgaeGrabberSubsystem(){
        angleMotor.getEncoder().setPosition(0);
        angleController.setReference(0, ControlType.kPosition);
    }

    @Override
    public void periodic() {
    }

    public void setPositionDegrees(double degrees){
        angleController.setReference(((degrees)*20)/360, ControlType.kPosition);
    }

    public double getPositionDegrees(){
        return angleMotor.getEncoder().getPosition()/CONVERSION_FACTOR;
    }

    public void setGrabberSpeed(double speed){
        grabberMotor.set(speed);
    }

    public double checkGrabberCurrent(){
        return grabberMotor.getOutputCurrent();
    }

    public Command spitAlgae(){
        return this.run(()->{
            grabberMotor.set(0.3);
        });
    }

    public Command resetAlgaeState(){
        return this.run(()->{
            angleController.setReference(0, ControlType.kPosition);
            grabberMotor.set(0);
        });
    }
}
