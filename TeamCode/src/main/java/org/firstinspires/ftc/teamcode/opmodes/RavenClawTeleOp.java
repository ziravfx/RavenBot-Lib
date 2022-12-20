package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.robot;

@TeleOp(name = "RavenClaw TeleOp", group = "OpModes") //change this!

public class RavenClawTeleOp extends LinearOpMode {

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
			double ly = -gamepad1.left_stick_y;
			double lx = gamepad1.left_stick_x;
			double rx = gamepad1.right_stick_x;
			double trigs = gamepad1.left_trigger - gamepad1.right_trigger;

			RavenBot.drive(ly, lx, rx); //drives our mecanum chassis
			RavenBot.armDrive(trigs); //drives our arm using trigger force

		}
	}
}
