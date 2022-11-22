package org.firstinspires.ftc.teamcode.drive.opmode.compOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

@Config
@TeleOp(name="TeleOp_Test", group="Linear Opmode")
public class ServoTest extends BaseOpMode {
    @Override
    public void runOpMode() {

        GetHardware();
        double servoTestValue = 0;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.dpad_right) {

                servoTestValue += .1;

            }

            if (gamepad1.dpad_left) {

                servoTestValue -= .1;

            }

            if (gamepad1.dpad_up) {

                servoTestValue += .01;

            }

            if (gamepad1.dpad_down) {

                servoTestValue -= .01;

            }

            //PID Code for arms
            /*controller.setPID(p, i, d);
            int vertArmPos = vertArm.getCurrentPosition();
            double pid = controller.calculate((vertArmPos), vertArmTarget);
            double ff = Math.cos(Math.toRadians(vertArmTarget / ticks_in_degrees)) * f;

            double vertArmPower = pid + ff;

            vertArm.setPower(vertArmPower);*/

            double y_stick = gamepad2.left_stick_y;
            double x_stick = gamepad2.left_stick_x;

            //Field Orientation Code
            double pi = 3.1415926;
            double gyro_degrees = navx_centered.getYaw();
            double gyro_radians = gyro_degrees * pi / 180;
            double temp = y_stick * Math.cos(gyro_radians) + x_stick * Math.sin(gyro_radians);
            x_stick = -y_stick * Math.sin(gyro_radians) + x_stick * Math.cos(gyro_radians);

            /* double temp = y_stick * Math.cos(gyro_radians) + x_stick * Math.sin(gyro_radians);
            x_stick = -y_stick * Math.sin(gyro_radians) + x_stick * Math.cos(gyro_radians); */ //Original code from last year

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

            //Show encoder values on the phone
            telemetry.addData("Status", "Initialized");

            telemetry.addData("Servo Test Angle", navx_centered.getYaw());

            //telemetry.addData("Vert Arm Pos", vertArmPos);
            //telemetry.addData("Vert Arm Target", vertArmTarget);
            telemetry.update();

            horizClaw.setPosition(servoTestValue);

        }
    }
}