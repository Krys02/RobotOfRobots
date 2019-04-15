/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Disabled;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareROR;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop Test Drive")
@Disabled
public class RORTeleopTestDrive extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareROR robot           = new HardwareROR();   // Use a Pushbot's hardware


    private ElapsedTime runtime = new ElapsedTime();

    double power;
    boolean powerToggle = true;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, this, telemetry, false, true);

        telemetry.addData("Say", "Ready for start");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        power = 0.5f;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (power > 1) power = 1;
            if (power < -1) power = -1;
            if (!gamepad1.dpad_up && !gamepad1.dpad_down){
                powerToggle = true;
            }
            if (gamepad1.dpad_up && powerToggle) {
                powerToggle = false;
                power += 0.1f;
            }
            if (gamepad1.dpad_down && powerToggle) {
                powerToggle = false;
                power -= 0.1f;
            }

            if (gamepad1.a) robot.backLeftDrive.setPower(power);
            else robot.backLeftDrive.setPower(0);

            if (gamepad1.b) robot.backRightDrive.setPower(power);
            else robot.backRightDrive.setPower(0);

            if (gamepad1.x) robot.frontLeftDrive.setPower(power);
            else robot.frontLeftDrive.setPower(0);

            if (gamepad1.y) robot.frontRightDrive.setPower(power);
            else robot.frontRightDrive.setPower(0);

            telemetry.addData("Power", power);
            telemetry.update();

            }
        }
    }
