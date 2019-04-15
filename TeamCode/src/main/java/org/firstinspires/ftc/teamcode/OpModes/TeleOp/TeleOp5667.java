package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import android.util.Log;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareROR;

@TeleOp(name = "TeleOp 5667")
public class TeleOp5667 extends LinearOpMode {

    HardwareROR robot = new HardwareROR();
    ElapsedTime runtime = new ElapsedTime();

    float scoringMax = 1f;

    @Override
    public void runOpMode() {
        double strafe, drive, rotate;
        boolean intakeDown = true, intakeSpinning = true, scoringLiftPowerOverride = false, intakeLiftPowerOverride = true, scoringLiftOverride = false;
        boolean intakeRotTog = true, intakeSpinTog = true, intakeStopTog = true, autoScoreToggle = true, intakeReturnToggle = true;
        int intakeSpinDir = 1;
        boolean automateScore = false, autoScoreRunningUp = false, autoScoreRunningDown = false, autoScoreTiltFor = false,
                autoScoreTiltBack = false, autoScoreWait = false, scoring = false;
        boolean automateIntakeReturn = false, autoIntakeReturning = false;
        int scoringLiftTargetTicks = 0, intakeLiftTargetTicks = 0;
        float scoringLiftPower, intakeLiftPower;
        float scoringTime = 1f;

        robot.init(hardwareMap, this, telemetry, false, true);

        robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
        while (!opModeIsActive() && !isStopRequested()) {
            robot.bucketCollect();
            telemetry.addData("Status:", "Waiting for start");
            telemetry.update();
        }

        robot.scoringLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();
        robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        while (opModeIsActive()) {
            if (runtime.seconds() > 3) {
                robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
            }
            // Drive Train
            strafe = gamepad1.left_stick_x;
            drive = -gamepad1.left_stick_y;
            rotate = gamepad1.right_trigger - gamepad1.left_trigger;

            robot.frontLeftDrive.setPower(drive + strafe + rotate);
            robot.frontRightDrive.setPower(drive - strafe - rotate);
            robot.backLeftDrive.setPower(drive - strafe + rotate);
            robot.backRightDrive.setPower(drive + strafe - rotate);

            if (gamepad2.a && intakeRotTog) {
                intakeRotTog = false;
                intakeDown = !intakeDown;
            }

            if (!gamepad2.a) {
                intakeRotTog = true;
            }

            if (intakeDown) {
                robot.intakeDown();
            } else if (!intakeDown) {
                robot.intakeUp();
            }

            if (gamepad2.b && intakeSpinTog) {
                intakeSpinTog = false;
                intakeSpinDir *= -1;
            }

            if (!gamepad2.b) {
                intakeSpinTog = true;
            }

            if (gamepad2.right_stick_button && intakeStopTog) {
                intakeSpinDir = 1;
                intakeStopTog = false;
                intakeSpinning = !intakeSpinning;
            }

            if (!gamepad2.right_stick_button) {
                intakeStopTog = true;
            }

            if (intakeSpinning) {
                robot.sweeperPower(intakeSpinDir);
            } else {
                robot.sweeperPower(0);
            }

            // Hang lift
            if (gamepad2.dpad_up) {
                robot.setHangLift(-1.0f);
            } else if (gamepad2.dpad_down) {
                robot.setHangLift(1.0f);
            } else {
                robot.setHangLift(0.0f);
            }


            // Scoring bucket
            if (!scoring) {
                if (gamepad1.a) {
                    robot.bucketScore();
                } else {
                    robot.bucketCollect();
                }
            }

            if (gamepad2.y && intakeReturnToggle) {
                intakeLiftTargetTicks = 0;
                automateIntakeReturn = true;
                intakeLiftPowerOverride = false;
                intakeDown = false;
            }
            if (automateIntakeReturn) {
                automateIntakeReturn = false;
                autoIntakeReturning = true;
            }
            if (autoIntakeReturning) {
                if (robot.intakeLift.getCurrentPosition() > -10) {
                    robot.gateOpen();
                }
            } else if (gamepad2.x) {
                robot.gateOpen();
            } else {
                robot.gateClose();
            }
            if (Math.abs(gamepad1.right_stick_y) != 0.0) {
                intakeLiftPowerOverride = true;
                autoIntakeReturning = false;
                automateIntakeReturn = false;
            }
            if (!gamepad2.y) {
                intakeReturnToggle = true;
            }

            if (!scoringLiftOverride) {
                // Begin Automate Scoring (x)
                if (gamepad1.x && autoScoreToggle && !scoring) {
                    autoScoreToggle = false;
                    automateScore = true;
                    Log.d("ROR: ", "automate score");
                    robot.bucketCollect();
                }
                //Abort Scoring Action
                if ((gamepad1.x) && autoScoreToggle && scoring) {
                    autoScoreToggle = false;
                    robot.bucketCollect();
                    automateScore = false;
                    autoScoreRunningUp = false;
                    autoScoreRunningDown = false;
                    autoScoreTiltBack = false;
                    autoScoreTiltFor = false;
                    autoScoreWait = false;
                    scoringLiftTargetTicks = 0;
                    scoring = false;
                    scoringLiftPowerOverride = false;
                    scoringMax = 0.7f;
                }
                if (automateScore) {
                    scoring = true;
                    automateScore = false;
                    scoringLiftTargetTicks = -800;
                    autoScoreRunningUp = true;
                    robot.bucketCollect();
                    runtime.reset();
                }
                if (autoScoreRunningUp) {
                    scoringMax = 1;
                    if (runtime.seconds() < 1.2) {
                        if (robot.scoringLift.getCurrentPosition() < -790) {
                            autoScoreTiltFor = true;
                            autoScoreRunningUp = false;
                        }
                    } else {
                        if (robot.scoringLift.getCurrentPosition() < -790) {
                            autoScoreTiltFor = true;
                            autoScoreRunningUp = false;
                            scoringLiftPowerOverride = false;
                        } else {
                            scoringLiftPowerOverride = true;
                            robot.runScoringLift(-1f);
                        }
                    }
                }
                if (autoScoreTiltFor) {
                    scoringLiftPowerOverride = true;
                    robot.runScoringLift(-0.8f);
                    runtime.reset();
                    autoScoreWait = true;
                    autoScoreTiltFor = false;
                }
                if (autoScoreWait) {
                    scoringLiftPowerOverride = true;
                    telemetry.addData("Waiting for Score", runtime.seconds());
                    robot.bucketScore();
                    if (runtime.seconds() > scoringTime) {
                        autoScoreTiltBack = true;
                        autoScoreWait = false;
                    }
                }
                if (autoScoreTiltBack) {
                    scoringLiftPowerOverride = false;
                    robot.bucketCollect();
                    autoScoreRunningDown = true;
                    runtime.reset();
                    autoScoreTiltBack = false;
                }
                if (autoScoreRunningDown) {
                    scoringMax = 0.7f;
                    if (runtime.seconds() > 0.25f) {
                        if (runtime.seconds() < 3f) {
                            scoringLiftTargetTicks = 0;
                            if (robot.scoringLift.getCurrentPosition() < 10 &&
                                    robot.scoringLift.getCurrentPosition() > -10) {
                                scoring = false;
                                autoScoreRunningDown = false;
                            }
                        } else {
                            robot.runScoringLift(0.8f);
                            if (robot.scoringLift.getCurrentPosition() < 10) {
                                scoring = false;
                                autoScoreRunningDown = false;
                            }
                        }
                    }
                }
            } else {
                robot.runScoringLift(gamepad2.left_stick_y);
            }

            // Scoring lift manual override
            if (Math.abs(gamepad2.left_stick_y) != 0) {
                scoringLiftOverride = true;
            }

            if (gamepad1.x) {
                scoringLiftOverride = false;
            }

            // Auto Score Toggle
            if (!gamepad1.x) {
                autoScoreToggle = true;
            }

            if (gamepad2.start) {
                robot.scoringLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.scoringLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (gamepad1.start) {
                robot.intakeLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.intakeLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            // RUN SCORING LIFT AT P DETERMINED POWER USING ENCODER TICKS
            scoringLiftPower = scoringLiftTarget(scoringLiftTargetTicks);
            telemetry.addData("Scoring lift power", scoringLiftPower);

            if (!scoringLiftPowerOverride && !scoringLiftOverride) {
                robot.runScoringLift(scoringLiftPower);
            }

            if (intakeLiftPowerOverride) {
                robot.setIntakeLiftPower(-gamepad1.right_stick_y);
            } else {
                // RUN INTAKE LIFT AT P DETERMINED POWER USING ENCODER TICKS
                intakeLiftPower = intakeLiftTarget(intakeLiftTargetTicks);
                telemetry.addData("Intake lift power", intakeLiftPower);
                robot.setIntakeLiftPower(intakeLiftPower);
            }

            telemetry.addData("Status", "Running");
            telemetry.addData((intakeSpinDir == 1 ? "Intaking" : "Outtaking"), "");
            telemetry.addData("Battery Voltage", robot.getBatteryVoltage());
            telemetry.addData("Bucket Pos", robot.scoringBucket.getPosition());
            telemetry.addData("Hang Lift Pos", robot.hangLift.getCurrentPosition());
            telemetry.addData("Scoring Lift Pos", robot.scoringLift.getCurrentPosition());
            telemetry.addData("Intake Lift Pos", robot.intakeLift.getCurrentPosition());
            telemetry.addData("Intake Lift Target", intakeLiftTargetTicks);
            telemetry.update();
        }
    }

    private float scoringLiftTarget(int targetPosition) {
        float power, position, gain = .015f, minPower = 0.4f, maxpower = scoringMax;
        int error = 10;

        position = robot.scoringLift.getCurrentPosition();

        if (position < targetPosition + error && position > targetPosition - error)
            power = 0.0f;             // no adjustment.
        else
            power = targetPosition - robot.scoringLift.getCurrentPosition();

        power = power * gain;

        if (power > maxpower)
            power = maxpower;
        if (power < -maxpower)
            power = -maxpower;
        if (power < minPower && power > 0)
            power = minPower;
        if (power > -minPower && power < 0)
            power = -minPower;

        return power;
    }

    private float intakeLiftTarget(int targetPosition) {
        float power, position, gain = .015f, minPower = 0.1f;
        int error = 10;

        position = robot.intakeLift.getCurrentPosition();

        if (position < targetPosition + error && position > targetPosition - error)
            power = 0.0f;             // no adjustment.
        else
            power = targetPosition - robot.intakeLift.getCurrentPosition();

        power = power * gain;

        if (power > 1)
            power = 1;
        if (power < -1)
            power = -1;
        if (power < minPower && power > 0)
            power = minPower;
        if (power > -minPower && power < 0)
            power = -minPower;

        return -power;
    }

}
