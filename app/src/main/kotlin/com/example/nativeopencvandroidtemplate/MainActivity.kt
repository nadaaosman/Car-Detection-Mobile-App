package com.example.nativeopencvandroidtemplate



import android.Manifest
import android.app.Activity
import android.content.pm.PackageManager
import android.os.Bundle
import androidx.core.app.ActivityCompat
import android.util.Log
import android.view.SurfaceView
import android.view.WindowManager
import android.widget.Button
import android.widget.Toast
import org.opencv.android.BaseLoaderCallback
import org.opencv.android.CameraBridgeViewBase
import org.opencv.android.LoaderCallbackInterface
import org.opencv.android.OpenCVLoader
import org.opencv.core.Mat
import java.io.BufferedReader
import java.io.DataOutputStream
import java.io.InputStreamReader
import java.net.HttpURLConnection
import java.net.URL




class MainActivity : Activity(), CameraBridgeViewBase.CvCameraViewListener2 {

    private var mOpenCvCameraView: CameraBridgeViewBase? = null
    var isLine: Double =-1.0;
    var endPoint: Double =1.0; //inital guess that the endpoint is the second point in the line
    var hough:Int=0;
    var preState:Double=0.0;
    var  firstDetect:Double=0.0;
    //guess how many lines wil be there?
    //val numRows = 20;

    //val l = Array(numRows) { FloatArray(4) } // 2D float array with numRows rows and 4 columns


    private val mLoaderCallback = object : BaseLoaderCallback(this) {
        override fun onManagerConnected(status: Int) {
            when (status) {
                LoaderCallbackInterface.SUCCESS -> {
                    Log.i(TAG, "OpenCV loaded successfully")

                    // Load native library after(!) OpenCV initialization
                    System.loadLibrary("native-lib")

                    mOpenCvCameraView!!.enableView()
                }
                else -> {
                    super.onManagerConnected(status)
                }
            }
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        Log.i(TAG, "called onCreate")
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        // Permissions for Android 6+
        ActivityCompat.requestPermissions(
            this@MainActivity,
            arrayOf(Manifest.permission.CAMERA),
            CAMERA_PERMISSION_REQUEST
        )

        setContentView(R.layout.activity_main)

        mOpenCvCameraView = findViewById<CameraBridgeViewBase>(R.id.main_surface)

        mOpenCvCameraView!!.visibility = SurfaceView.VISIBLE

        mOpenCvCameraView!!.setCvCameraViewListener(this)
        val btnGet = findViewById<Button>(R.id.btn_get)

        btnGet.setOnClickListener {
            hough=1;
        }
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<String>,
        grantResults: IntArray
    ) {
        when (requestCode) {
            CAMERA_PERMISSION_REQUEST -> {
                if (grantResults.isNotEmpty() && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                    mOpenCvCameraView!!.setCameraPermissionGranted()
                } else {
                    val message = "Camera permission was not granted"
                    Log.e(TAG, message)
                    Toast.makeText(this, message, Toast.LENGTH_LONG).show()
                }
            }
            else -> {
                Log.e(TAG, "Unexpected permission request")
            }
        }
    }

    override fun onPause() {
        super.onPause()
        if (mOpenCvCameraView != null)
            mOpenCvCameraView!!.disableView()
    }

    override fun onResume() {
        super.onResume()
        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization")
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION, this, mLoaderCallback)
        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!")
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS)
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        if (mOpenCvCameraView != null)
            mOpenCvCameraView!!.disableView()
    }

    override fun onCameraViewStarted(width: Int, height: Int) {}

    override fun onCameraViewStopped() {}

    //    lateinit var lines: Array<IntArray>
    lateinit var lines: Array<DoubleArray>

    override fun onCameraFrame(frame: CameraBridgeViewBase.CvCameraViewFrame): Mat {
        // get current camera frame as OpenCV Mat object
        val mat = frame.rgba()

        // native call to process current camera frame

        if(hough==1) {
            Log.e(TAG, "button scope!!");

            //drawLinesFromJNI(mat.nativeObjAddr,lines)


            var myArray = adaptiveThresholdFromJNI(mat.nativeObjAddr, isLine, endPoint,lines)
            isLine = myArray[0];
            endPoint = myArray[1];


            Log.e(TAG, "//////////////////////////");
            Log.e(TAG, isLine.toString());
            Log.e(TAG, preState.toString());

            if(firstDetect!=0.0){
                //if not the same as previous take an action
                if(isLine!=preState){
                    preState=isLine;
                    if(isLine==-1.0){
                        //call speed down esp
                        sendData(0);
                        Log.e(TAG, "car isn't on line");
                    }
                    else{
                        Log.e(TAG, "car is on line");
                        sendData(1);
                        //call speed up
                    }
                }
                else
                {
                    Log.e(TAG, "Same state");

                }
            }
            else{
                Log.d(TAG, "DONE FIRST DETECTION!")
                if(isLine!=-1.0){
                    //call speed up
                    Log.e(TAG, "car is on line");

                }
                preState=isLine;
                firstDetect=1.0;
            }



//        if (checkLines == 0) {
//            var myArray = adaptiveThresholdFromJNI(mat.nativeObjAddr, isLine, endPoint, 0,)
//            checkLines = 1;
//            isLine = myArray[0];
//            endPoint = myArray[1];
//            if (isLine != -1) {
//                Log.e(TAG, "car is on line");
//                //esp send signal 1

//            } else {
//                Log.e(TAG, "car isn't on line");
//                //esp send signal 0
//            }
//            checkLines=1;
//        } else {
//            drawLinesFromJNI(mat.nativeObjAddr,lines)
//            var myArray = adaptiveThresholdFromJNI(mat.nativeObjAddr, isLine, endPoint, 1)
//            isLine = myArray[0];
//            endPoint = myArray[1];
//            if (isLine != -1) {
//                Log.e(TAG, "car is on line");
//
//                //esp send signal 1
//            } else {
//                Log.e(TAG, "car isn't on line");
//                //esp send signal 0
//            }
//        }
        }
        else{
            Log.e(TAG, "capture state!!");
            //get hough lines then draw it
            lines= getHoughFromJNI(mat.nativeObjAddr)
        //
        }
        // return processed frame for live preview
        return mat
    }



    private fun sendData(signal:Int ) {
        val ipAddress = "172.28.108.8"// Replace with the IP address of your ESP32
        val port = 80 // Replace with the port number of your server
//        print the URL
        val url = "http://$ipAddress:$port/post" // Construct the URL
        val postData = "signal=$signal"
        Thread {
            val conn = URL(url).openConnection() as HttpURLConnection
            conn.requestMethod = "POST"
            conn.doOutput = true
            conn.setRequestProperty("Content-Type", "application/x-www-form-urlencoded")
            conn.setRequestProperty("Content-Length", postData.length.toString())
            conn.useCaches = false
            println(postData)
            DataOutputStream(conn.outputStream).use { outputStream ->
                outputStream.writeBytes(postData)
                outputStream.flush() // ensure that data is immediately sent to the output stream

            }
//            println("nada")
            BufferedReader(InputStreamReader(conn.inputStream)).use { br ->
                var line: String?
                while (br.readLine().also { line = it } != null) {
                    println(line)
                }
            }

            conn.disconnect()
        }.start()
    }

    private external fun adaptiveThresholdFromJNI(matAddr: Long,isLine:Double,endPoint:Double,lines:Array<DoubleArray>): DoubleArray
    private external fun drawLinesFromJNI(matAddr: Long, lines: Array<DoubleArray>)
    private external fun getHoughFromJNI(matAddr: Long): Array<DoubleArray>


    companion object {

        private const val TAG = "MainActivity"
        private const val CAMERA_PERMISSION_REQUEST = 1
    }
}