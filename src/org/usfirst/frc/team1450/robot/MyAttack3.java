package org.usfirst.frc.team1450.robot;

import edu.wpi.first.wpilibj.Joystick;

public class MyAttack3 extends Joystick {

	public MyAttack3(int port) {
		super(port);
		// TODO Auto-generated constructor stub
	}
	
	public double GetZAsThrottle()
	{
		
		double output = (this.getZ() - 1.0) / (-2.0);
    	return output;
	}

}
