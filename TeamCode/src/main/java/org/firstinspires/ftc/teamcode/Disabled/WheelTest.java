package org.firstinspires.ftc.teamcode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareROR;

@TeleOp(name = "Location Test")
@Disabled
public class WheelTest extends LinearOpMode {

    HardwareROR robot = new HardwareROR();
    RORDetection detect = new RORDetection();
    float power = 0.2f;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this, telemetry, false, true);
        detect.init(hardwareMap, this, telemetry, robot, false);

        waitForStart();



        while (opModeIsActive()) {

            detect.goldLocationStr();

            /*
            if(gamepad1.right_stick_y > 0.1) {
                power -= 0.000005;
            } else if(gamepad1.right_stick_y < -0.1) {
                power += 0.000005;
            }

            if (gamepad1.a) {
                robot.frontLeftDrive.setPower(power);
            } else {
                robot.frontLeftDrive.setPower(0);
            }

            if (gamepad1.b) {
                robot.frontRightDrive.setPower(power);
            } else {
                robot.frontRightDrive.setPower(0);
            }

            if (gamepad1.x) {
                robot.backLeftDrive.setPower(power);
            } else {
                robot.backLeftDrive.setPower(0);
            }

            if (gamepad1.y) {
                robot.backRightDrive.setPower(power);
            } else {
                robot.backRightDrive.setPower(0);
            }

            telemetry.addData("Power", power);
            telemetry.update();
            */
        }

    }
}
