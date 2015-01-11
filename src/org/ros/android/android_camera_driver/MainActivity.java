/*
 * Copyright (C) 2011 Google Inc.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ros.android.android_camera_driver;

import android.content.Intent;
import android.hardware.Camera;
import android.os.AsyncTask;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.SubMenu;
import android.view.SurfaceView;
import android.view.Window;
import android.view.WindowManager;

import com.google.common.base.Preconditions;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.exception.RosRuntimeException;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.net.URI;
import java.net.URISyntaxException;

/**
 * @author chadrockey@gmail.com (Chad Rockey)
 * @author axelfurlan@gmail.com (Axel Furlan)
 * @author tal.regev@gmail.com  (Tal Regev)
 */


public class MainActivity extends RosActivity {
    private static final String TAG = "android_camera_driver::MainActivity";

    public static final int VIEW_MODE_RGBA = 0;
    public static final int VIEW_MODE_GRAY = 1;
    public static final int VIEW_MODE_CANNY = 2;
    public static final int IMAGE_TRANSPORT_COMPRESSION_NONE = 0;
    public static final int IMAGE_TRANSPORT_COMPRESSION_PNG = 1;
    public static final int IMAGE_TRANSPORT_COMPRESSION_JPEG = 2;


    public static int viewMode = VIEW_MODE_RGBA;
    public static int imageCompression = IMAGE_TRANSPORT_COMPRESSION_JPEG;

    public static int imageJPEGCompressionQuality = 80;
    public static int imagePNGCompressionQuality = 3;

    public static int mCameraId = 0;

    private final int MASTER_CHOOSER_REQUEST_CODE = 1;
    private CameraBridgeViewBase mOpenCvCameraView;
    private CameraPublisher cam_pub = new CameraPublisher();
    private Boolean isInit = false;
    private String robotName;

    @SuppressWarnings("deprecation")
    protected final int numberOfCameras = Camera.getNumberOfCameras();

    protected BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS: {
                    Log.i(TAG, "OpenCV loaded successfully");
                    mOpenCvCameraView.enableView();
                }
                break;
                default: {
                    super.onManagerConnected(status);
                }
                break;
            }
        }
    };


    public MainActivity() {
        super("ROS Camera Driver", "ROS Camera Driver");
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.main);
        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.HelloOpenCvView);
        mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
        mOpenCvCameraView.setCvCameraViewListener(cam_pub);
    }

    @Override
    public void startMasterChooser() {
        Preconditions.checkState(getMasterUri() == null);
        // Call this method on super to avoid triggering our precondition in the
        // overridden startActivityForResult().
        Intent intent = new Intent(this, MasterChooser.class);
        super.startActivityForResult(intent, MASTER_CHOOSER_REQUEST_CODE);
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        if (resultCode == RESULT_OK) {
            if (requestCode == MASTER_CHOOSER_REQUEST_CODE) {
                if (data.getBooleanExtra("ROS_MASTER_CREATE_NEW", false)) {
                    nodeMainExecutorService.startMaster(data.getBooleanExtra("ROS_MASTER_PRIVATE", true));
                }
                else {
                    URI uri;
                    try {
                        uri = new URI(data.getStringExtra("ROS_MASTER_URI"));
                        robotName = data.getStringExtra("ROBOT_NAME");
                    }
                    catch (URISyntaxException e) {
                        throw new RosRuntimeException(e);
                    }
                    nodeMainExecutorService.setMasterUri(uri);
                }
                // Run init() in a new thread as a convenience since it often requires network access.
                new AsyncTask<Void, Void, Void>() {
                    @Override
                    protected Void doInBackground(Void... params) {
                        MainActivity.this.init(nodeMainExecutorService);
                        return null;
                    }
                }.execute();
            }
            else {
                // Without a master URI configured, we are in an unusable state.
                nodeMainExecutorService.shutdown();
            }
        }
    }

    @Override
    public void onResume() {
        super.onResume();
        if (isInit) {
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_9, this, mLoaderCallback);
        }
    }

    @Override
    public void onPause() {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {

        SubMenu subPreview = menu.addSubMenu("Color settings");
        subPreview.add(1, VIEW_MODE_RGBA, 0, "RGB Color").setChecked(true);
        subPreview.add(1, VIEW_MODE_GRAY, 0, "Gray Scale");
        subPreview.add(1, VIEW_MODE_CANNY, 0, "Canny edges");
        subPreview.setGroupCheckable(1, true, true);

        SubMenu subCompression = menu.addSubMenu("Compression");
        subCompression.add(2,IMAGE_TRANSPORT_COMPRESSION_NONE,0,"None");
        SubMenu subPNGCompressionRate = subCompression.addSubMenu(2, IMAGE_TRANSPORT_COMPRESSION_PNG, 0, "Png");
        subPNGCompressionRate.setHeaderTitle("Compression quality");
        subPNGCompressionRate.getItem().setChecked(true);
        subPNGCompressionRate.add(4, 3, 0, "3").setChecked(true);
        subPNGCompressionRate.add(4, 4, 0, "4");
        subPNGCompressionRate.add(4, 5, 0, "5");
        subPNGCompressionRate.add(4, 6, 0, "6");
        subPNGCompressionRate.add(4, 7, 0, "7");
        subPNGCompressionRate.add(4, 8, 0, "8");
        subPNGCompressionRate.add(4, 9, 0, "9");
        subPNGCompressionRate.setGroupCheckable(4, true, true);

        SubMenu subJPEGCompressionRate = subCompression.addSubMenu(2, IMAGE_TRANSPORT_COMPRESSION_JPEG, 0, "Jpeg");
        subCompression.setGroupCheckable(2, true, true);
        subJPEGCompressionRate.setHeaderTitle("Compression quality");
        subJPEGCompressionRate.getItem().setChecked(true);
        subJPEGCompressionRate.add(3, 50, 0, "50");
        subJPEGCompressionRate.add(3, 60, 0, "60");
        subJPEGCompressionRate.add(3, 70, 0, "70");
        subJPEGCompressionRate.add(3, 80, 0, "80").setChecked(true);
        subJPEGCompressionRate.add(3, 90, 0, "90");
        subJPEGCompressionRate.add(3, 100, 0, "100");
        subJPEGCompressionRate.setGroupCheckable(3, true, true);

        if (numberOfCameras > 1) {
            menu.addSubMenu(5, 0, 0, "Swap camera");
        }

        return super.onCreateOptionsMenu(menu);
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getGroupId()) {
            case 1:
                viewMode = item.getItemId();
                item.setChecked(true);
                break;
            case 2:
                imageCompression = item.getItemId();
                item.setChecked(true);
                break;
            case 3:
                imageJPEGCompressionQuality = item.getItemId();
                item.setChecked(true);
                break;
            case 4:
                imagePNGCompressionQuality = item.getItemId();
                item.setChecked(true);
                break;
            case 5:
                swapCamera();
                break;
        }
        return super.onOptionsItemSelected(item);
    }

    //from http://stackoverflow.com/questions/16273370/opencvandroidsdk-switching-between-front-camera-and-back-camera-at-run-time
    public void swapCamera() {
        mCameraId = (mCameraId + 1) % numberOfCameras;
        mOpenCvCameraView.disableView();
        mOpenCvCameraView.setCameraIndex(mCameraId);
        mOpenCvCameraView.enableView();
    }


    protected void init(NodeMainExecutor nodeMainExecutor) {
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());

        nodeConfiguration.setMasterUri(getMasterUri());
        nodeConfiguration.setNodeName("android_camera_driver");

        nodeMainExecutor.execute(cam_pub, nodeConfiguration);
        cam_pub.robotName = robotName;
        Log.i(TAG, "called nodeMainExecutor");
        isInit = true;
        onResume();
    }
}
