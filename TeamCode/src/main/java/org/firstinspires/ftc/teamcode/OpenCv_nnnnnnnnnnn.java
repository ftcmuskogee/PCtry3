package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@Autonomous
public class OpenCv_nnnnnnnnnnn extends OpMode {
    OpenCvWebcam Webcam1;

    @Override
    public void init() {

        //WebcamName webcamName = hardwareMap.get(WebcamName.class,"Webcam1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        Webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        Webcam1.setPipeline(new examplePipline());

        Webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                Webcam1.startStreaming(640,360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }
    @Override
    public void loop(){

    }
    class examplePipline extends OpenCvPipeline{
        Mat YCbCr = new Mat();
        Mat leftcrop;
        Mat rightcrop;
        Mat midcrop;
        Mat midcrop2;
        double leftavgfin;
        double rightavgfin;
        double midavgfin;
        double midavgfin2;
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(255.0,0.0,0.0);


        public Mat processFrame(Mat input){

        Imgproc.cvtColor(input,YCbCr,Imgproc.COLOR_RGB2YCrCb);
        telemetry.addLine("pipeline running");

        Rect leftRect = new Rect(1,1,319,359);
        Rect midRect = new Rect(160,1,319,359);
        Rect midRect2 = new Rect(160,1,319,359);
        Rect rightRect = new Rect(320,1,319,359);


        input.copyTo(outPut);
        Imgproc.rectangle(outPut,leftRect,rectColor,2);
        Imgproc.rectangle(outPut,rightRect,rectColor,2);
        Imgproc.rectangle(outPut,midRect,rectColor,2);
        //Imgproc.rectangle(outPut,midRect2,rectColor,2);

        leftcrop = YCbCr.submat(leftRect);
        rightcrop = YCbCr.submat(rightRect);
        midcrop = YCbCr.submat(midRect);
        midcrop2 = YCbCr.submat(midRect2);

        Core.extractChannel(leftcrop,leftcrop,2);
        Core.extractChannel(rightcrop,rightcrop,2);
        Core.extractChannel(midcrop,midcrop,2);
        Core.extractChannel(midcrop2,midcrop2,2);

        Scalar leftavg = Core.mean(leftcrop);
        Scalar rightavg = Core.mean(rightcrop);
        Scalar midavg = Core.mean(rightcrop);
        Scalar midavg2 = Core.mean(rightcrop);

        leftavgfin = leftavg.val[0];
        rightavgfin = rightavg.val[0];
        midavgfin = midavg.val[0];
        midavgfin2 = midavg2.val[0];

        if (leftavgfin > midavgfin) {
            telemetry.addLine("left");
            if (leftavgfin > rightavgfin )
                telemetry.addLine("Left1");
                if (leftavgfin > midavgfin2 )
                    telemetry.addLine("Left1");
        }
        else if (rightavgfin > midavgfin) {
            telemetry.addLine("right");
            if (midavgfin > leftavgfin)
                telemetry.addLine("kindaRight");
                if (midavgfin > leftavgfin)
                    telemetry.addLine("actuallyRight");

        }
        else{
            telemetry.addLine("Middle");
        }

        return(outPut);
        }

        }
    }


