package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.robot;

@TeleOp(name = "Method Testing", group = "Testing") //change this!
//@Disabled //comment this out to enable your opmode. *VERY IMPORTANT, YOUR OPMODE WILL NOT SHOW UP OTHERWISE.*
public class MethodTesting extends LinearOpMode {

	//adding our robot object
	robot RavenBot = new robot(this);

	@Override
	public void runOpMode() throws InterruptedException {
		//code that runs on initialization
		RavenBot.init(); //gets all our hardware, whatever
		RavenBot.IMUinit(); //initializes our IMU hardware

		double maxPwr = .75;

		waitForStart();

		//code that runs ONCE when start is pressed (variables that don't need to update, etc.)

		while(opModeIsActive() && !isStopRequested()){
			//code that runs continuously once the opmode has started.
			RavenBot.forward(maxPwr, 12);
			RavenBot.right(maxPwr, 12);
			RavenBot.turn(maxPwr, -90);
			RavenBot.absTurn(maxPwr, 90);
			RavenBot.forward(maxPwr, 12);
		}

	}
}


