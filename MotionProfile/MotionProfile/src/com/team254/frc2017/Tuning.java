package com.team254.frc2017;

public class Tuning {
	public double findDriveTrainScrubFacter(double numberOfTurns, double centerToCenterDistanceOfWheels, 
			 double encoderTicks, double inchesPerEncoderTick){
				return (numberOfTurns * centerToCenterDistanceOfWheels * Math.PI)/(encoderTicks * inchesPerEncoderTick);
		
	}
	public double findInchesPerEncoderTick(double encoderTicks, double distance){
		return distance/encoderTicks;
	}
	public double findEncoderTicksPerInches(double encoderTicks, double distance){
		return distance/encoderTicks;
	}
}

