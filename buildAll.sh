# Bash

../gradlew clean debug
if [ $? -eq 0 ]
 then
  adb uninstall org.chadrockey.android.android_camera_driver
  adb install bin/MainActivity-debug.apk
  adb shell am start -n org.chadrockey.android.android_camera_driver/org.chadrockey.android.android_camera_driver.MainActivity
  adb logcat -c
  adb logcat
fi
