/*
 * Copyright (c) 2011, Chad Rockey
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Android Sensors Driver nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package org.ros.android.android_camera_driver;

import android.graphics.Bitmap;
import android.hardware.Camera;
import android.util.Log;

import org.jboss.netty.buffer.ChannelBufferOutputStream;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfInt;
import org.opencv.highgui.Highgui;
import org.opencv.imgproc.Imgproc;
import org.ros.internal.message.MessageBuffers;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import java.nio.ByteBuffer;

import sensor_msgs.CameraInfo;
import sensor_msgs.CompressedImage;
import sensor_msgs.Image;

import static org.opencv.core.Core.flip;


/**
 * @author chadrockey@gmail.com (Chad Rockey)
 * @author axelfurlan@gmail.com (Axel Furlan)
 * @author tal.regev@gmail.com  (Tal Regev)
 */

public class CameraPublisher implements NodeMain, CvCameraViewListener2 {
    private static final String TAG = "SENSORS::CameraPublisher";


    protected ConnectedNode node = null;
    protected Publisher<CompressedImage> imagePublisher;
    protected Publisher<sensor_msgs.Image> rawImagePublisher;
    protected Publisher<sensor_msgs.CameraInfo> cameraInfoPublisher;
    protected sensor_msgs.CameraInfo cameraInfo;
    protected int counter;
    @SuppressWarnings("deprecation")
    protected Camera.CameraInfo info = new Camera.CameraInfo();
    @SuppressWarnings("deprecation")
    protected final int FRONT = Camera.CameraInfo.CAMERA_FACING_FRONT;
    public String robotName;


    public CameraPublisher() {
    }


    public GraphName getDefaultNodeName() {
        return GraphName.of("android_camera_driver/cameraPublisher");
    }

    public void onError(Node node, Throwable throwable) {
    }

    public void onStart(final ConnectedNode node) {
        this.node = node;
        NameResolver resolver;
        resolver = node.getResolver().newChild(robotName + "/camera");
        imagePublisher = node.newPublisher(resolver.resolve("image/compressed"), CompressedImage._TYPE);
        cameraInfoPublisher = node.newPublisher(resolver.resolve("camera_info"), CameraInfo._TYPE);
        rawImagePublisher = node.newPublisher(resolver.resolve("image/raw"), Image._TYPE);
        Log.i(TAG, "called onStart");
    }

    @Override
    public void onShutdown(Node arg0) {

    }

    @Override
    public void onShutdownComplete(Node arg0) {
    }

    @Override
    public void onCameraViewStarted(int width, int height) {
    }

    @Override
    public void onCameraViewStopped() {
    }


    @Override
    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
        Mat         frame       = null;
        Mat         frameToSend = new Mat();
        MatOfByte   buf         = new MatOfByte();
        MatOfInt    params;

        counter = (counter + 1) % 7;
        ChannelBufferOutputStream stream = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
        switch (MainActivity.viewMode) {
            case MainActivity.VIEW_MODE_RGBA:
                frame = inputFrame.rgba();
                break;
            case MainActivity.VIEW_MODE_GRAY:
            case MainActivity.VIEW_MODE_CANNY:
                frame = inputFrame.gray();
                break;
        }
        if (null != node && null != frame) {

            //noinspection deprecation
            android.hardware.Camera.getCameraInfo(MainActivity.mCameraId, info);
            if (info.facing == FRONT)
            {
                flip(frame, frame,1);
            }

            switch (MainActivity.viewMode) {
                case MainActivity.VIEW_MODE_RGBA:
                    Imgproc.cvtColor(frame, frameToSend, Imgproc.COLOR_RGBA2BGR);
                    break;
                case MainActivity.VIEW_MODE_GRAY:
                    frameToSend = frame;
                    break;
                case MainActivity.VIEW_MODE_CANNY:
                    Imgproc.Canny(frame, frame, 80, 100);
                    frameToSend = frame;
                    break;
            }

            Time currentTime = node.getCurrentTime();

            int cols = frame.cols();
            int rows = frame.rows();

            try {

                cameraInfo = cameraInfoPublisher.newMessage();
                cameraInfo.getHeader().setFrameId("camera");
                cameraInfo.getHeader().setStamp(currentTime);
                cameraInfo.setWidth(cols);
                cameraInfo.setHeight(rows);
                cameraInfoPublisher.publish(cameraInfo);


                if (MainActivity.imageCompression >= MainActivity.IMAGE_TRANSPORT_COMPRESSION_PNG)
                {
                    //Compressed image
                    CompressedImage image = imagePublisher.newMessage();

                    image.getHeader().setStamp(currentTime);
                    image.getHeader().setFrameId("camera");

                    switch (MainActivity.imageCompression)
                    {
                        case MainActivity.IMAGE_TRANSPORT_COMPRESSION_PNG:
                            image.setFormat("png");
                            params = new MatOfInt(Highgui.IMWRITE_PNG_COMPRESSION, MainActivity.imageJPEGCompressionQuality);
                            Highgui.imencode(".png", frameToSend, buf, params);
                            break;
                        case MainActivity.IMAGE_TRANSPORT_COMPRESSION_JPEG:
                            image.setFormat("jpeg");
                            params = new MatOfInt(Highgui.IMWRITE_JPEG_QUALITY, MainActivity.imageJPEGCompressionQuality);
                            Highgui.imencode(".jpg", frameToSend, buf, params);
                            break;

                    }

                    stream.write(buf.toArray());
                    image.setData(stream.buffer().copy());
                    imagePublisher.publish(image);
                }
                else if (counter == 0)
                {
                    // Raw image
                    Log.i(TAG, "Raw image 1");


                    Bitmap bmp = Bitmap.createBitmap(cols, rows, Bitmap.Config.ARGB_8888);
                    Utils.matToBitmap(frame, bmp);


                    Image rawImage = rawImagePublisher.newMessage();
                    rawImage.getHeader().setStamp(currentTime);
                    rawImage.getHeader().setFrameId("camera");
                    rawImage.setEncoding("rgba8");
                    rawImage.setWidth(cols);
                    rawImage.setHeight(rows);
                    rawImage.setStep(bmp.getRowBytes());

                    ByteBuffer bb = ByteBuffer.allocate(bmp.getRowBytes() * bmp.getHeight());


                    bmp.copyPixelsToBuffer(bb);
                    stream.write(bb.array());
                    rawImage.setData(stream.buffer().copy());
                    rawImagePublisher.publish(rawImage);

                    /*
                    params = new MatOfInt(Highgui.CV_IMWRITE_PXM_BINARY, 1);
                    Highgui.imencode(".bmp", frameToSend, buf, params);
                    stream.write(buf.toArray());
                    rawImage.setData(stream.buffer().copy());
                    rawImagePublisher.publish(rawImage);
                    stream.buffer().clear();
                    */
                    Log.i(TAG, "Raw image 6");
                }
            }
            catch (Exception e)
            {
                Log.e(TAG, "Frame conversion and publishing throws an exception: " + e.getMessage());
            }
        }
        return frame;
    }
}