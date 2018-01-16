package com.team254.lib.util.drivers;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU overhead by skipping duplicate set
 * commands. (By default the Talon flushes the Tx buffer on every set call).
 */
public class LazyCANTalon extends TalonSRX {
    protected double mLastSet = Double.NaN;
 

    public LazyCANTalon(int deviceNumber) {
        super(deviceNumber);
    }

    
}
