package com.team254.lib.util.drivers;

import edu.wpi.first.wpilibj.MotorSafety;


import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.*;

/**
 * Creates CANTalon objects and configures all the parameters we care about to factory defaults. Closed-loop and sensor
 * parameters are not set, as these are expected to be set by the application.
 */
public class CANTalonFactory {

    public static class Configuration {
        public boolean LIMIT_SWITCH_NORMALLY_OPEN = true;
        public double MAX_OUTPUT_VOLTAGE = 12;
        public double NOMINAL_VOLTAGE = 0;
        public double PEAK_VOLTAGE = 12;
        public NeutralMode ENABLE_BRAKE = NeutralMode.Brake;
        public boolean ENABLE_CURRENT_LIMIT = false;
        public boolean ENABLE_SOFT_LIMIT = false;
        public boolean ENABLE_LIMIT_SWITCH = false;
        public int CURRENT_LIMIT = 0;
        public double EXPIRATION_TIMEOUT_SECONDS = MotorSafety.DEFAULT_SAFETY_EXPIRATION;
        public double FORWARD_SOFT_LIMIT = 0;
        public boolean INVERTED = false;
        public double NOMINAL_CLOSED_LOOP_VOLTAGE = 12;
        public double REVERSE_SOFT_LIMIT = 0;
        public boolean SAFETY_ENABLED = false;

        public int CONTROL_FRAME_PERIOD_MS = 5;
        public int MOTION_CONTROL_FRAME_PERIOD_MS = 100;
        public int GENERAL_STATUS_FRAME_RATE_MS = 5;
        public int FEEDBACK_STATUS_FRAME_RATE_MS = 100;
        public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 100;
        public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 100;
        public int PULSE_WIDTH_STATUS_FRAME_RATE_MS = 100;

        public VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD = VelocityMeasPeriod.Period_100Ms;
        public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

        public double VOLTAGE_COMPENSATION_RAMP_RATE = 0;
        public double VOLTAGE_RAMP_RATE = 0;
    }

    private static final Configuration kDefaultConfiguration = new Configuration();
    private static final Configuration kSlaveConfiguration = new Configuration();

    static {
        kSlaveConfiguration.CONTROL_FRAME_PERIOD_MS = 1000;
        kSlaveConfiguration.MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
        kSlaveConfiguration.GENERAL_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
    }

    // Create a CANTalon with the default (out of the box) configuration.
    public static TalonSRX createDefaultTalon(int id) {
        return createTalon(id, kDefaultConfiguration);
    }

    public static TalonSRX createPermanentSlaveTalon(int id, TalonSRX master_id) {
        final TalonSRX talon1 = createTalon(id, kSlaveConfiguration);
        talon1.follow(master_id);
        return talon1;
    }

    public static TalonSRX createTalon(int id, Configuration config) {
        TalonSRX talon = new LazyCANTalon(id);
        talon.changeMotionControlFramePeriod(config.MOTION_CONTROL_FRAME_PERIOD_MS);
        talon.setIntegralAccumulator(0, 0, 10);
        talon.clearMotionProfileHasUnderrun(10);
        talon.clearMotionProfileTrajectories();
        talon.clearStickyFaults(10);
        talon.configForwardSoftLimitEnable(config.LIMIT_SWITCH_NORMALLY_OPEN, 10);
        talon.configNominalOutputForward(config.NOMINAL_VOLTAGE, 10);
        talon.configNominalOutputReverse(-config.NOMINAL_VOLTAGE, 10);
        talon.configPeakOutputForward(1, 10);
        talon.configPeakOutputReverse(-1, 10);
        talon.setNeutralMode(config.ENABLE_BRAKE);
        talon.enableCurrentLimit(config.ENABLE_CURRENT_LIMIT);
        talon.enableCurrentLimit(config.ENABLE_SOFT_LIMIT);
        talon.setInverted(false);
        talon.setSensorPhase(false);
        talon.getSensorCollection().setAnalogPosition(0, 10);
        talon.configContinuousCurrentLimit(config.CURRENT_LIMIT, 10);
        talon.setInverted(config.INVERTED);
        talon.configNominalOutputForward(config.NOMINAL_CLOSED_LOOP_VOLTAGE, 0);
        talon.configNominalOutputReverse(-1 * config.NOMINAL_CLOSED_LOOP_VOLTAGE, 0);
        talon.setSelectedSensorPosition(0, 0, 0);
        talon.selectProfileSlot(0, 10);
        talon.getSensorCollection().setPulseWidthPosition(0, 10);
        talon.configForwardSoftLimitEnable(config.ENABLE_LIMIT_SWITCH, 10);
        talon.configReverseSoftLimitEnable(config.ENABLE_LIMIT_SWITCH, 10);
        talon.configVelocityMeasurementPeriod(config.VELOCITY_MEASUREMENT_PERIOD, 10);
        talon.configVelocityMeasurementWindow(config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW, 10);
        talon.configOpenloopRamp(config.VOLTAGE_COMPENSATION_RAMP_RATE, 10);
        talon.configClosedloopRamp(config.VOLTAGE_COMPENSATION_RAMP_RATE, 10);

        talon.setStatusFramePeriod(StatusFrame.Status_1_General, config.GENERAL_STATUS_FRAME_RATE_MS, 10);
        talon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, config.FEEDBACK_STATUS_FRAME_RATE_MS, 10);
        talon.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, config.FEEDBACK_STATUS_FRAME_RATE_MS, 10);

        return talon;
    }

}
