package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeGrabberSubsystem extends SubsystemBase {
    private final SparkFlex angleMotor = new SparkFlex(30, MotorType.kBrushless);
    private final SparkClosedLoopController angleController = angleMotor.getClosedLoopController();
    private final SparkFlex grabberMotor = new SparkFlex(31, MotorType.kBrushless);
    public static final double CONVERSION_FACTOR = 20.0/360.0;
    public AlgaeGrabberSubsystem(){
        SparkFlexConfig angleMotorConfig = new SparkFlexConfig();
        angleMotorConfig.closedLoop
        .p(1)
        .i(0)
        .d(0)
        .outputRange(-0.1, 1);
        angleMotor.configure(angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        angleMotor.getEncoder().setPosition(0);
        angleController.setReference(0, ControlType.kPosition);
        
        SparkFlexConfig grabberMotorConfig = new SparkFlexConfig();
        grabberMotorConfig.smartCurrentLimit(10);
        grabberMotor.configure(grabberMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
    }

    public void setPositionDegrees(double degrees){
        angleController.setReference(((degrees)*20)/360, ControlType.kPosition);
    }

    public void setGrabberCurrentLimit(int currentLimit){
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(currentLimit);
        grabberMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public double getPositionDegrees(){
        return angleMotor.getEncoder().getPosition()/CONVERSION_FACTOR;
    }

    public void setGrabberSpeed(double speed){
        grabberMotor.set(speed);
    }

    public double checkGrabberRPM(){
        return grabberMotor.getEncoder().getVelocity();
    }

    public Command spitAlgae(){
        return this.runEnd(()->{
            grabberMotor.set(1);
            setGrabberCurrentLimit(40);
        },
        ()->{
            grabberMotor.stopMotor();
            setGrabberCurrentLimit(10);
        });
    }

    public Command resetAlgaeState(){
        return this.run(()->{
            angleController.setReference(0, ControlType.kPosition);
            grabberMotor.set(0);
        });
    }
}
