package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.mechanisms.robot;

@TeleOp(name = "IMUTest", group = "Testing") //change this!
//@Disabled //comment this out to enable your opmode. *VERY IMPORTANT, YOUR OPMODE WILL NOT SHOW UP OTHERWISE.*
public class IMUTest extends LinearOpMode {

	//adding our robot object

	IMU imu;
	robot RavenBot = new robot(this);

	private void turn(double Pwr, float Deg){
		RavenBot.forward(Pwr, Deg);
	}

	@Override
	public void runOpMode() throws InterruptedException {
		//code that runs on initialization
		RavenBot.init(); //gets all our hardware, whatever
		RavenBot.IMUinit();
		waitForStart();

		//code that runs ONCE when start is pressed (variables that don't need to update, etc.)
		while(opModeIsActive() && !isStopRequested()){
			turn(1, 5);
			//code that runs continuously once the opmode has started.
			RavenBot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
			telemetry.addData("imu", RavenBot.yaw());
			telemetry.update();

		}

	}
}



