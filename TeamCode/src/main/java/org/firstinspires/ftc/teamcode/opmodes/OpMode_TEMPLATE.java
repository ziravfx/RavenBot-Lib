package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.robot;

@TeleOp(name = "Opmode Template", group = "OpModes") //change this!
@Disabled //comment this out to enable your opmode. *VERY IMPORTANT, YOUR OPMODE WILL NOT SHOW UP OTHERWISE.*
public class OpMode_TEMPLATE extends LinearOpMode {

	//adding our robot object
	robot RavenBot = new robot(this);

	@Override
	public void runOpMode() throws InterruptedException {
		//code that runs on initialization
		RavenBot.init(); //gets all our hardware, whatever

		waitForStart();

		//code that runs ONCE when start is pressed (variables that don't need to update, etc.)

		while(opModeIsActive() && !isStopRequested()){
			//code that runs continuously once the opmode has started.

		}

	}
}


