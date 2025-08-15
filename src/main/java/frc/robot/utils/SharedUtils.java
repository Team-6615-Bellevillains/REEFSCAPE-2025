package frc.robot.utils;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Current;

public class SharedUtils {
    public static void setCurrentLimit(SparkFlex motor, Current currentLimit) {
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit((int)currentLimit.in(Amps));
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public static void setCurrentLimit(SparkMax motor, Current currentLimit) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit((int)currentLimit.in(Amps));
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}