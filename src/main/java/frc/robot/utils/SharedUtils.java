package frc.robot.utils;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SharedUtils {
    public static void setCurrentLimit(SparkFlex motor, int currentLimit) {
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(currentLimit);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public static void setCurrentLimit(SparkMax motor, int currentLimit) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(currentLimit);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}