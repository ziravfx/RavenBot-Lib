package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.dependencies.CMYSleeveDetection;
import org.firstinspires.ftc.teamcode.mechanisms.robot;

@Autonomous(name = "Zone Park", group = "OpModes") //change this!
// @Disabled comment this out to enable your opmode. *VERY IMPORTANT, YOUR OPMODE WILL NOT SHOW UP OTHERWISE.*
public class ZonePark extends LinearOpMode {

	//adding our robot object
	robot RavenBot = new robot(this);

	@Override
	public void runOpMode() throws InterruptedException {
		//code that runs on initialization
		RavenBot.init(); //gets all our hardware, whatever
		RavenBot.OCVinit();

		while(!isStarted()){
			telemetry.addData("Parking Position", RavenBot.getParkPos());
			telemetry.update();
		}

		double maxPwr = .75;

		waitForStart();

		//code that runs ONCE when start is pressed (variables that don't need to update, etc.)
		CMYSleeveDetection.ParkingPosition parkPos = RavenBot.getParkPos();

		while(opModeIsActive() && !isStopRequested()){
			//code that runs continuously once the opmode has started.
			switch(parkPos){
				case LEFT:
					RavenBot.left(maxPwr, 23.85f);
					RavenBot.forward(maxPwr, 23.85f);
					break;
				case CENTER:
					RavenBot.forward(maxPwr, 23.85f);
					break;
				case RIGHT:
					RavenBot.right(maxPwr, 23.85f);
					RavenBot.forward(maxPwr, 23.85f);
					break;
			}
			sleep(30000);
		}

	}
}


