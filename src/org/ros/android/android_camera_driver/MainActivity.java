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

import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.SubMenu;
import android.view.Window;
import android.view.WindowManager;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

/**
 * @author chadrockey@gmail.com (Chad Rockey)
 * @author axelfurlan@gmail.com (Axel Furlan)
 */


public class MainActivity extends RosActivity {

    public static final int VIEW_MODE_RGB = 0;
    public static int viewMode = VIEW_MODE_RGB;
    public static final int VIEW_MODE_GRAY = 1;
    public static final int VIEW_MODE_CANNY = 2;
    public static final int IMAGE_TRANSPORT_COMPRESSION_NONE = 0;
    public static final int IMAGE_TRANSPORT_COMPRESSION_PNG = 1;
    public static final int IMAGE_TRANSPORT_COMPRESSION_JPEG = 2;
    public static int imageCompression = IMAGE_TRANSPORT_COMPRESSION_JPEG;
    // OpenCV Camera
    private static final String TAG = "SENSORS::MainActivity";
    public static int imageJPEGCompressionQuality = 80;
    public static int imagePNGCompressionQuality = 80;
    private CameraPublisher cam_pub;


    public MainActivity() {
        super("ROS Camera Driver", "ROS Camera Driver");
    }

    @Override
    protected void onPause() {
        super.onPause();
        if (cam_pub != null) {
            cam_pub.releaseCamera();
        }
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN);
        setContentView(R.layout.main);
    }

    @Override
    protected void onResume() {
        super.onResume();

        if (cam_pub != null) {
            cam_pub.resume();
        } else
            Log.i(TAG, "Error while resuming app");
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {

        SubMenu subPreview = menu.addSubMenu("Color settings");
        subPreview.add(1, VIEW_MODE_RGB, 0, "RGB Color").setChecked(true);
        subPreview.add(1, VIEW_MODE_GRAY, 0, "Gray Scale");
        subPreview.add(1, VIEW_MODE_CANNY, 0, "Canny edges");
        subPreview.setGroupCheckable(1, true, true);

        SubMenu subCompression = menu.addSubMenu("Compression");
        //subCompression.add(2,IMAGE_TRANSPORT_COMPRESSION_NONE,0,"None");
        SubMenu subPNGCompressionRate = subCompression.addSubMenu(2, IMAGE_TRANSPORT_COMPRESSION_PNG, 0, "Png");
        subPNGCompressionRate.setHeaderTitle("Compression quality");
        subPNGCompressionRate.getItem().setChecked(true);
        subPNGCompressionRate.add(4, 50, 0, "50");
        subPNGCompressionRate.add(4, 60, 0, "60");
        subPNGCompressionRate.add(4, 70, 0, "70");
        subPNGCompressionRate.add(4, 80, 0, "80").setChecked(true);
        subPNGCompressionRate.add(4, 90, 0, "90");
        subPNGCompressionRate.add(4, 100, 0, "100");
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
        }
        return super.onOptionsItemSelected(item);
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
        nodeConfiguration.setMasterUri(getMasterUri());
        nodeConfiguration.setNodeName("android_camera_driver");
        this.cam_pub = new CameraPublisher(this);
        nodeMainExecutor.execute(this.cam_pub, nodeConfiguration);
    }
}
