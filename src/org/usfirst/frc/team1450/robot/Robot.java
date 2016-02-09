
package org.usfirst.frc.team1450.robot;


import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.StatusFrameRate;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * WARNING: While it may look like a good choice to use for your code if you're inexperienced,
 * don't. Unless you know what you are doing, complex code will be much more difficult under
 * this system. Use IterativeRobot or Command-Based instead if you're new.
 */
public class Robot extends SampleRobot {
    CANTalon motor0;
    CANTalon motor1;
    CANTalon motor2;
    CANTalon motor3;
    CANTalon motor4;
    CANTalon motor5;
    DigitalInput isLow;
    DigitalInput isTurning;
    DoubleSolenoid tiltSolenoid;
    CameraServer camServ;
    ADXRS450_Gyro gyro;
    double feetToSeconds;
    boolean doPID = false;
    
	RobotDrive robotDrive;

	Servo camServoX;
	Servo camServoY;
	
    MyAttack3 stick3;
    MyAttack3 stick4;
    int velProfile = 0;
    int posProfile = 1;

    public Robot() {
        stick3 = new MyAttack3(0);
        stick4 = new MyAttack3(1);
        motor0 = new CANTalon(10);
        motor1 = new CANTalon(11);
        motor2 = new CANTalon(12);
        motor3 = new CANTalon(13);
        motor4 = new CANTalon(14);
        motor5 = new CANTalon(15);
        tiltSolenoid = new DoubleSolenoid(2, 3);
        isLow = new DigitalInput(9);
        isTurning = new DigitalInput(8);
        camServoX = new Servo(0);
        gyro = new ADXRS450_Gyro();
       
        camServoY = new Servo(1);
    	
        motor1.changeControlMode(TalonControlMode.Follower);
        motor1.set(motor0.getDeviceID());
        motor3.changeControlMode(TalonControlMode.Follower);
        motor3.set(motor2.getDeviceID());
        motor4.ConfigFwdLimitSwitchNormallyOpen(true);
        motor4.ConfigRevLimitSwitchNormallyOpen(true);
        motor4.enableLimitSwitch(true, true);
        motor5.ConfigFwdLimitSwitchNormallyOpen(true);
        motor5.ConfigRevLimitSwitchNormallyOpen(true);
        motor5.enableLimitSwitch(true, true);
        if (doPID)
        {
        	motor0.changeControlMode(TalonControlMode.Speed);
            motor0.setFeedbackDevice(FeedbackDevice.QuadEncoder);
            motor0.setPosition(0);
            motor2.setPosition(0);
            motor0.set(0);
            motor2.set(0);
            motor0.setProfile(0);
            motor2.setProfile(0);
            double p = 0.1;
            SmartDashboard.putNumber("propGain", p);
            double i = 0.002;
            double d = 0.0;
            double f = 0.0;
            int iZone = 0;
            double rampRate = 6;
            motor0.setPID(p, i, d, f, iZone, rampRate, velProfile);
			motor0.setStatusFrameRateMs(StatusFrameRate.QuadEncoder, 10);
			motor0.reverseSensor(true);
            motor2.changeControlMode(TalonControlMode.Speed);
            motor2.setFeedbackDevice(FeedbackDevice.QuadEncoder);
            motor2.setPID(p, i, d, f, iZone, rampRate, velProfile);
            motor2.setStatusFrameRateMs(StatusFrameRate.QuadEncoder, 10);
            SmartDashboard.putNumber("velScale", 700);
            SmartDashboard.putNumber("intGain", i);
            motor2.reverseOutput(true);
            SmartDashboard.putBoolean("DoPosition", false);
            p = 0.7;
            i = 0.0025;
            motor0.setPID(p, i, d, f, iZone, rampRate, posProfile);
            motor2.setPID(p, i, d, f, iZone, rampRate, posProfile);
            robotDrive = new RobotDrive(motor0, motor2);
        }
        else
        {
        	robotDrive = new RobotDrive(motor0, motor2);
            robotDrive.setMaxOutput(1);
			motor0.reverseOutput(false);
			motor1.reverseOutput(false);
			motor2.reverseOutput(false);
			motor3.reverseOutput(false);
			//method no longer supported
//            robotDrive.setInvertedMotor(MotorType.kFrontLeft,true);
//            robotDrive.setInvertedMotor(MotorType.kFrontRight,true);
//            robotDrive.setInvertedMotor(MotorType.kRearLeft,true);
//            robotDrive.setInvertedMotor(MotorType.kRearRight,true);
        }
        
    	camServ = CameraServer.getInstance();
        try
        {
	        camServ.setQuality(50);
	        camServ.startAutomaticCapture("cam0");
	        
        }
        catch (Exception e)
        {
        	//
        }
	        
    }

    /**
     * Drive left & right motors for 2 seconds then stop
     */
    public void autonomous() {
    	AutoCode2();
    }

    public void AutoCode1()
    {
    	feetToSeconds = 2.0/3.0;
        robotDrive.setSafetyEnabled(false);
        robotDrive.drive(-0.25, 0.0);	// drive forwards half speed
        Timer.delay(3.0 * feetToSeconds);		//    for 2 seconds = 3ft
        motor0.enableBrakeMode(true);
        motor1.enableBrakeMode(true);
        motor2.enableBrakeMode(true);
        motor3.enableBrakeMode(true);
        robotDrive.drive(0.0, 0.0);	// stop robot
        motor4.set(-1.0);
        Timer.delay(2.5);
        motor4.set(0.0);
        motor0.enableBrakeMode(false);
        motor1.enableBrakeMode(false);
        motor2.enableBrakeMode(false);
        motor3.enableBrakeMode(false);
        robotDrive.drive(-0.5, 1.0);	// drive forwards half speed
        Timer.delay(2.1 * feetToSeconds/*1.4*/);
        motor0.enableBrakeMode(true);
        motor1.enableBrakeMode(true);
        motor2.enableBrakeMode(true);
        motor3.enableBrakeMode(true);
        robotDrive.drive(0.0, 0.0);	// stop robot
        robotDrive.drive(-0.25, 0.0);	// drive forwards half speed
        Timer.delay(6.0 * feetToSeconds);
        motor0.enableBrakeMode(true);
        motor1.enableBrakeMode(true);
        motor2.enableBrakeMode(true);
        motor3.enableBrakeMode(true);
        robotDrive.drive(0.0, 0.0);	// stop robot
    }
    
    public void AutoCode2()
    {
    	feetToSeconds = 2.0/3.0;
        robotDrive.setSafetyEnabled(false);
        //Lift
        motor4.set(1.0);
        if (isLow.get())
        {
        	Timer.delay(1.125);
        }
        else
        {
        	Timer.delay(1.5);
        }
        motor4.set(0.0);
        //Turn
//        if (isTurning.get())
//        {
//	        if (isRight.get())
//	        {
//	        	robotDrive.drive(-0.5, 1.0);	// turn right
//	        }
//	        else
//	        {
//	        	robotDrive.drive(0.5, 1.0);		// turn left
//	        }
//	        Timer.delay(2.525 * 90.0 / 180.0 * feetToSeconds/*1.4*/);
//	        motor0.enableBrakeMode(true);
//	        motor1.enableBrakeMode(true);
//	        motor2.enableBrakeMode(true);
//	        motor3.enableBrakeMode(true);
//	        robotDrive.drive(0.0, 0.0);	// stop robot
//	        motor0.enableBrakeMode(false);
//	        motor1.enableBrakeMode(false);
//	        motor2.enableBrakeMode(false);
//	        motor3.enableBrakeMode(false);
//        }
        //Drive
        robotDrive.drive(-0.25, 0.0);	// drive forwards half speed
//        if (isTurning.get())
//        {
//        	Timer.delay(5.0 * feetToSeconds);
//        }
//        else
//        {
        	Timer.delay(7.0 * feetToSeconds);
//        }
        motor0.enableBrakeMode(true);
        motor1.enableBrakeMode(true);
        motor2.enableBrakeMode(true);
        motor3.enableBrakeMode(true);
        robotDrive.drive(0.0, 0.0);	// stop robot
        motor0.enableBrakeMode(false);
        motor1.enableBrakeMode(false);
        motor2.enableBrakeMode(false);
        motor3.enableBrakeMode(false);
    }
    
    /**
     * Runs the motors with arcade steering.
     */
    double lowPassFilteredSpeed = 0.0;
    
    double ThrottleCalc(double input)
    {
    	double output = (input - 1.0) / (-2.0);
    	return output;
    }
   
    
    public void operatorControl() {
    	if (!doPID)
    	{
    		robotDrive.setSafetyEnabled(true);
    	}
        motor4.enableLimitSwitch(true, true);
        while (isOperatorControl() && isEnabled()) {
        	System.out.println(motor0.getEncPosition());
        	System.out.println(motor2.getEncPosition());
        	SmartDashboard.putNumber("LeftEncoder",motor0.getEncPosition() * -1);
        	SmartDashboard.putNumber("RightEncoder",motor2.getEncPosition());
        	//max velocity 7000Hz
        	SmartDashboard.putNumber("LeftVelocity",-1 * motor0.getEncVelocity());
        	SmartDashboard.putNumber("RightVelocity",motor2.getEncVelocity());
            //robotDrive.arcadeDrive(stick,true); // drive with arcade style (use right stick)
//        	lowPassFilteredSpeed += ((stick1.getY() * ThrottleCalc(stick1.getZ())) - lowPassFilteredSpeed) * 0.3; 
//            robotDrive.arcadeDrive(lowPassFilteredSpeed /** 0.7*/, stick1.getX() * ThrottleCalc(stick1.getZ()),true);
            lowPassFilteredSpeed += ((stick3.getY() * stick3.GetZAsThrottle()) - lowPassFilteredSpeed) * 0.3; 
            if (!doPID)
            {
            	robotDrive.arcadeDrive(-lowPassFilteredSpeed /** 0.7*/, -stick3.getX() * stick3.GetZAsThrottle(),true);
            	camServoX.set(((stick4.getY()*-1)+1)/2);
            	camServoY.set((stick4.getX()+1)/2);
            	SmartDashboard.putNumber("GyroAngle", gyro.getAngle());
            }
            else
            {
            	motor0.set(/*1 * SmartDashboard.getNumber("velScale")*/6000 * stick3.GetZAsThrottle());
            	motor2.set(/*1 * SmartDashboard.getNumber("velScale")*/6000 * stick3.GetZAsThrottle());
            	//max ~6000
            	SmartDashboard.putNumber("velCmd", stick3.GetZAsThrottle() * SmartDashboard.getNumber("velScale"));
            	SmartDashboard.putNumber("closedLoopError0", motor0.getClosedLoopError());
            	SmartDashboard.putNumber("compOut0", motor0.getOutputVoltage());
            	SmartDashboard.putNumber("closedLoopError2", motor2.getClosedLoopError());
            	SmartDashboard.putNumber("compOut2", motor2.getOutputVoltage());
            	if (stick3.getRawButton(10))
                {
                	double p = SmartDashboard.getNumber("propGain");
                    double i = SmartDashboard.getNumber("intGain");
                    double d = 0.0;
                    double f = 0.0;
                    int iZone = 0;
                    int profile = 0;
                    if (SmartDashboard.getBoolean("DoPosition"))
                    {
                    	profile = posProfile;
                    }
                    double rampRate = 0.0;
                    motor0.setPID(p, i, d, f, iZone, rampRate, profile);            
                    motor2.setPID(p, i, d, f, iZone, rampRate, profile);
                    motor0.ClearIaccum();
                    motor2.ClearIaccum();
            		if (SmartDashboard.getBoolean("DoPosition"))
            		{
            			motor0.changeControlMode(TalonControlMode.Position);
            			motor2.changeControlMode(TalonControlMode.Position);
            			motor0.setPosition(0);
            			motor2.setPosition(0);
            			motor0.setProfile(posProfile);
            			motor2.setProfile(posProfile);
            		}
            		else
            		{
            			motor0.changeControlMode(TalonControlMode.Speed);
            			motor2.changeControlMode(TalonControlMode.Speed);
            			motor0.setProfile(velProfile);
            			motor2.setProfile(velProfile);
            		}
                }
            }
            //robotDrive.arcadeDrive(stick.getY(), stick.getX());
//            double secondStick = stick2.getX() * ThrottleCalc(stick2.getZ());
            double secondStick = stick4.getX() * stick4.GetZAsThrottle();
            
            if ((secondStick > 0.25) || (secondStick < -0.25)) 
            {
            	motor5.set(secondStick);
            }
            else
            {
            	motor5.set(0.0);
            }
			if (stick4.getRawButton(1))
    		{
    			motor4.set(1.0 * stick4.GetZAsThrottle());	//up
    		}
    		else
    		{
    			if (stick4.getRawButton(2))
        		{
    				motor4.set(-1.0 * stick4.GetZAsThrottle());	// down
        		}
        		else
        		{
        			motor4.set(0.0);
        		}            		
    		} 
			if (stick4.getRawButton(3))
    		{
				tiltSolenoid.set(Value.kForward);
    		}
    		else
    		{
    			if (stick4.getRawButton(4))
        		{
    				tiltSolenoid.set(Value.kReverse);
        		}
        		else
        		{
        			tiltSolenoid.set(Value.kOff);
        		}            		
    		}
            Timer.delay(0.005);		// wait for a motor update time
        }
    }

    /**
     * Runs during test mode
     */
    public void test() {
    }
}
