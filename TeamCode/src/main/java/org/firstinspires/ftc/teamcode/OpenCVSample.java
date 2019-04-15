package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Rect;

public class OpenCVSample {

    public GoldDetector detector;

    private Telemetry t;
    private HardwareMap hwMap;

    private int screenWidth = 640;
    private String center = "CENTER", left = "LEFT", right = "RIGHT", none = "NONE";
    public String currentGoldPos = none;
    public String lastGoldPos = none;

    public OpenCVSample () {   }

    protected void init(HardwareMap ahwMap, LinearOpMode opMode, Telemetry telemetry){
        t = telemetry;
        hwMap = ahwMap;

        detector = new GoldDetector();
        detector.init(hwMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();
        detector.downscale = 0.4;
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        detector.maxAreaScorer.weight = 0.005;
        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;
        detector.enable();
    }

    protected void sample() {
        Rect rect = detector.getFoundRect();
        int goldXPos = (int) (rect.x + rect.width * 0.5);
        if (detector.isFound()) {
            t.addData("GoldX: ", goldXPos);
            if (goldXPos < screenWidth / 3) {
                currentGoldPos = left;
                lastGoldPos = left;
            } else if (goldXPos >= screenWidth / 3 && goldXPos < (screenWidth / 3) * 2) {
                currentGoldPos = center;
                lastGoldPos = center;
            } else if (goldXPos >= (screenWidth / 3) * 2) {
                currentGoldPos = right;
                lastGoldPos = right;
            }
            t.addData("OpenCV", "Tracking gold at position " + currentGoldPos);
        } else {
            currentGoldPos = none;
            t.addData("OpenCV", "Gold not found, last seen at position " + lastGoldPos);
        }
    }

}
