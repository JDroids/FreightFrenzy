package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/*
 * See https://github.com/OpenFTC/EasyOpenCV/blob/master/examples/src/main/java/org/firstinspires/ftc/teamcode/SkystoneDeterminationExample.java
 */
public class BarcodeTeamShippingElementPipeline extends OpenCvPipeline {
    private Point region1Pos;
    private Point region2Pos;
    private Point region3Pos;

    private static final int REGION_WIDTH = 20;
    private static final int REGION_HEIGHT = 20;

    private static final Scalar BLUE = new Scalar(0, 0, 255);
    private static final Scalar GREEN = new Scalar(0, 255, 0);

    private Rect leftRectangle;
    private Rect centerRectangle;
    private Rect rightRectangle;

    private Mat YCrCb = new Mat();
    private Mat cB = new Mat();

    private Mat leftRegion;
    private Mat centerRegion;
    private Mat rightRegion;

    private Randomization randomization;

    public BarcodeTeamShippingElementPipeline(Point region1Pos,
                                              Point region2Pos,
                                              Point region3Pos) {
        this.region1Pos = region1Pos;
        this.region2Pos = region2Pos;
        this.region3Pos = region3Pos;
    }

    @Override
    public void init(Mat mat) {
        leftRectangle = new Rect(
                region1Pos,
                new Point(
                        region1Pos.x + REGION_WIDTH,
                        region1Pos.y + REGION_HEIGHT));

        centerRectangle = new Rect(
                region2Pos,
                new Point(
                        region2Pos.x + REGION_WIDTH,
                        region2Pos.y + REGION_HEIGHT));

        rightRectangle = new Rect(
                region3Pos,
                new Point(
                        region3Pos.x + REGION_WIDTH,
                        region3Pos.y + REGION_HEIGHT));

        inputToCb(mat);

        leftRegion = cB.submat(leftRectangle);
        centerRegion = cB.submat(centerRectangle);
        rightRegion = cB.submat(rightRectangle);
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat matToShow = cB;

        inputToCb(input);

        Imgproc.rectangle(matToShow, leftRectangle, BLUE);
        Imgproc.rectangle(matToShow, centerRectangle, BLUE);
        Imgproc.rectangle(matToShow, rightRectangle, BLUE);

        double leftAverage = Core.mean(leftRegion).val[0];
        double centerAverage = Core.mean(centerRegion).val[0];
        double rightAverage = Core.mean(rightRegion).val[0];

        double min = Math.min(leftAverage, Math.min(centerAverage, rightAverage));

        if (min == leftAverage) {
            randomization = Randomization.LEVEL_1;
        }
        else if (min == centerAverage) {
            randomization = Randomization.LEVEL_2;
        }
        else if (min == rightAverage) {
            randomization = Randomization.LEVEL_3;
        }

        Imgproc.putText(matToShow, "" + randomization, new Point(0, 50), Imgproc.FONT_HERSHEY_PLAIN, 1.0, GREEN);

        return matToShow;
    }

    public Randomization getRandomization() {
        return randomization;
    }

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'cB' variable
     */
    private void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, cB, 2);
    }

    public enum Randomization {
        LEVEL_1,
        LEVEL_2,
        LEVEL_3
    }
}
