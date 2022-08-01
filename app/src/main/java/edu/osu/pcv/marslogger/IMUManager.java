package edu.osu.pcv.marslogger;

import android.app.Activity;
import android.content.Context;
import android.content.SharedPreferences;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Process;
import android.preference.PreferenceManager;
import android.util.Log;

import com.marslogger.locationprovider.LocationProvider;

import java.io.BufferedWriter;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Deque;
import java.util.Iterator;

import timber.log.Timber;

public class IMUManager implements SensorEventListener {
    private static final String TAG = "IMUManager";
    // if the accelerometer data has a timestamp within the
    // [t-x, t+x] of the gyro data at t, then the original acceleration data
    // is used instead of linear interpolation
    private final long mInterpolationTimeResolution = 500; // nanoseconds
    private int mSensorRate = SensorManager.SENSOR_DELAY_FASTEST;

    // needed to synchronize gps and imu values for the same timestamp
    private final GPSManager gpsManager;

    public static String ImuHeader = "Timestamp[nanosec],gx[rad/s],gy[rad/s],gz[rad/s]," +
            "ax[m/s^2],ay[m/s^2],az[m/s^2],Unix time[nanosec]\n";

    private class SensorPacket {
        long timestamp; // nanoseconds
        long unixTime; // milliseconds
        float[] values;

        SensorPacket(long time, long unixTimeMillis, float[] vals) {
            timestamp = time;
            unixTime = unixTimeMillis;
            values = vals;
        }

        @Override
        public String toString() {
            String delimiter = ",";
            StringBuilder sb = new StringBuilder();
            sb.append(timestamp);
            for (int index = 0; index < values.length; ++index) {
                sb.append(delimiter + values[index]);
            }
            sb.append(delimiter + unixTime + "000000");
            return sb.toString();
        }
    }

    // Sensor listeners
    private SensorManager mSensorManager;
    private Sensor mAccel;
    private Sensor mGyro;
    private static SharedPreferences mSharedPreferences;
    private int linear_acc; // accuracy
    private int angular_acc;

    private volatile boolean mRecordingInertialData = false;
    private FileOutputStream fos = null;
    private BufferedWriter mDataWriter = null;
    private HandlerThread mSensorThread;

    private Deque<SensorPacket> mGyroData = new ArrayDeque<>();
    private Deque<SensorPacket> mAccelData = new ArrayDeque<>();

    public IMUManager(Activity activity, GPSManager gpsManager) {
        this.gpsManager = gpsManager;
        mSensorManager = (SensorManager) activity.getSystemService(Context.SENSOR_SERVICE);
        mAccel = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        mGyro = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        mSharedPreferences = PreferenceManager.getDefaultSharedPreferences(activity);
    }

    public void startRecording(String captureResultFile) {
        try {
            fos = new FileOutputStream(captureResultFile);
            mDataWriter = new BufferedWriter(new OutputStreamWriter(fos));
            if (mGyro == null || mAccel == null) {
                String warning = "The device may not have a gyroscope or an accelerometer!\n" +
                        "No IMU data will be logged.\n" +
                        "Has Gyroscope? " + (mGyro == null ? "No" : "Yes") + "\n"
                        + "Has Accelerometer? " + (mAccel == null ? "No" : "Yes") + "\n";
                mDataWriter.write(warning);
            } else {
                mDataWriter.write(ImuHeader);
            }
            mRecordingInertialData = true;
        } catch (IOException err) {
            Timber.e(err, "IOException in opening inertial data writer at %s",
                    captureResultFile);
        }
    }

    public void stopRecording() {
        if (mRecordingInertialData) {
            mRecordingInertialData = false;
            try {
                mDataWriter.flush();
//                mDataWriter.close();
//                fos.flush();
//                fos.close();
            } catch (IOException err) {
                Timber.e(err, "IOException in closing inertial data writer");
            }
            mDataWriter = null;
//            fos = null;
        }
    }

    @Override
    public final void onAccuracyChanged(Sensor sensor, int accuracy) {
        if (sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            linear_acc = accuracy;
        } else if (sensor.getType() == Sensor.TYPE_GYROSCOPE) {
            angular_acc = accuracy;
        }
    }


    @Override
    public final void onSensorChanged(SensorEvent event) {
        long unixTime = System.currentTimeMillis();
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            if (mRecordingInertialData) {
                try {
                    StringBuilder sb = new StringBuilder();
                    sb.append("1,");
                    for (float v : event.values) {
                        sb.append(v + ",");
                    }
                    sb.append(event.timestamp).append(unixTime).append("\n");
                    mDataWriter.write(sb.toString());
                } catch (IOException ioe) {
                    Timber.e(ioe);
                }
            }
        } else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {

            if (mRecordingInertialData) {
                try {
                    StringBuilder sb = new StringBuilder();
                    sb.append("2,");
                    for (float v : event.values) {
                        sb.append(v + ",");
                    }
                    sb.append(event.timestamp).append(unixTime).append("\n");
                    mDataWriter.write(sb.toString());
                } catch (IOException ioe) {
                    Timber.e(ioe);
                }
            }
        }
    }

    /**
     * This will register all IMU listeners
     * https://stackoverflow.com/questions/3286815/sensoreventlistener-in-separate-thread
     */
    public void register() {
        mSensorThread = new HandlerThread("Sensor thread",
                Process.THREAD_PRIORITY_MORE_FAVORABLE);
        mSensorThread.start();
        String imuFreq = mSharedPreferences.getString("prefImuFreq", "1");
        mSensorRate = Integer.parseInt(imuFreq);
        // Blocks until looper is prepared, which is fairly quick
        Handler sensorHandler = new Handler(mSensorThread.getLooper());
        mSensorManager.registerListener(
                this, mAccel, mSensorRate, sensorHandler);
        mSensorManager.registerListener(
                this, mGyro, mSensorRate, sensorHandler);
    }

    /**
     * This will unregister all IMU listeners
     */
    public void unregister() {
        mSensorManager.unregisterListener(this, mAccel);
        mSensorManager.unregisterListener(this, mGyro);
        mSensorManager.unregisterListener(this);
        mSensorThread.quitSafely();
        gpsManager.unregister();
        stopRecording();
    }
}
