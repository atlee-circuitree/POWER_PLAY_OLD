package org.firstinspires.ftc.teamcode.drive.opmode.compOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.Bases.BaseOpMode;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="hambunger", group="Linear Opmode")
public class TeleOP_2022_2023 extends BaseOpMode {

    @Override
    public void runOpMode() {

        GetHardware();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Show encoder values on the phone
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Left Dead Encoder", frontLeft.getCurrentPosition());
            telemetry.addData("Right Dead Encoder", rearRight.getCurrentPosition());
            telemetry.addData("Rear Dead Encoder", rearLeft.getCurrentPosition());
            telemetry.addData("HorizArm Amount Extended", horizArm.getCurrentPosition());
            telemetry.addData("VertArm Amount Extended", horizArm.getCurrentPosition());
            telemetry.addData("HorizArm Amount Extended", horizArm.getCurrentPosition());
            telemetry.update();

            double horizPower = PIDControl(1000, horizArm.getVelocity()); //reference = ticks per second

            double y_stick = gamepad2.left_stick_y;
            double x_stick = gamepad2.left_stick_x;

            //Field Orientation Code
            double pi = 3.1415926;
            double gyro_degrees = navx_centered.getYaw();
            double gyro_radians = gyro_degrees * pi/180;
            double temp = y_stick * Math.cos(gyro_radians) + x_stick * Math.sin(gyro_radians);
            x_stick = -y_stick * Math.sin(gyro_radians) + x_stick * Math.cos(gyro_radians);

            //Mecanum Drive Code
            double r = Math.hypot(x_stick, y_stick);
            double robotAngle = Math.atan2(y_stick, -x_stick) - Math.PI / 4;
            double rightX = -gamepad2.right_stick_x;
            final double v1 = (r * Math.cos(robotAngle) + rightX);
            final double v2 = (r * Math.sin(robotAngle) - rightX);
            final double v3 = (r * Math.sin(robotAngle) + rightX);
            final double v4 = (r * Math.cos(robotAngle) - rightX);

            frontLeft.setPower(v1 * SD);
            rearLeft.setPower(v3 * SD);
            frontRight.setPower(v2 * SD);
            rearRight.setPower(v4 * SD);



            //Controller 1 Auto Tele-op
            //Does arm movements and extensions automatically
            /*if (gamepad1.a) {
                horizArm.setPower(horizPower);
            } else if (gamepad1.a) {
                horizArm.setPower(horizPower);
                }
            }*/


            //Controller 2 Manual Tele-op
            //Slows movement
            if (gamepad2.left_stick_button) {
                SD = .25;
            } else {
                SD = 1;
            }

            //Resets NavX heading
            if (gamepad2.back) {
                zeroGyro();
            }

            //Extends and Retracts horizArm
            if (gamepad2.x) {
                horizArm.setPower(.2);
            } else if (gamepad2.a) {
                horizArm.setPower(-.2);
            } else {
                horizArm.setPower(0);
            }

            //Opens and Closes claw
            if (gamepad2.y) {
                horizClaw.setPosition(.2);
            } else if (gamepad2.b) {
                horizClaw.setPosition(-.2);
            } else {
                horizClaw.setPosition(0);
            }

            //Moves vertArm up and down
            if (gamepad2.dpad_up){
                vertArm.setPower(.2);
            } else if (gamepad2.dpad_down) {
                vertArm.setPower(-.2);
            } else {
                vertArm.setPower(0);
            }

            //Opens and Closes Transfer Claw
            if (gamepad2.left_bumper) {
                transferClaw.setPosition(.2);
            } else if (gamepad2.right_bumper) {
                transferClaw.setPosition(-.2);
            } else {
                transferClaw.setPosition(0);
            }

            //Moves angleArm up and down
            if (gamepad2.right_trigger > .5 ) {
                angleArm.setPower(.2);
            } else if (gamepad2.left_trigger > .5) {
                angleArm.setPower(-.2);
            } else {
                angleArm.setPower(0);
            }
        }
    }
}