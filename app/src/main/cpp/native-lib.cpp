#include <jni.h>
#include <android/log.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#define TAG "NativeLib"

using namespace std;
using namespace cv;

//function for checking the intersection the circle with a line segment
vector<int> solve_circle_line_segment_intersection(Point2f center, double radius, Point line_p1, Point line_p2) {
    vector<int> result(2, 0);

    // Calculate the coefficients of the quadratic equation
    double a = pow(line_p2.x - line_p1.x, 2) + pow(line_p2.y - line_p1.y, 2);
    double b = 2 * ((line_p2.x - line_p1.x) * (line_p1.x - center.x) + (line_p2.y - line_p1.y) * (line_p1.y - center.y));
    double c = pow(center.x, 2) + pow(center.y, 2) + pow(line_p1.x, 2) + pow(line_p1.y, 2) - 2 * (center.x * line_p1.x + center.y * line_p1.y) - pow(radius, 2);
//    __android_log_print(ANDROID_LOG_DEBUG, TAG, "Line %d %d %d %d ", line_p1.x,line_p1.y,line_p2.x,line_p2.y );

    // Calculate the discriminant of the quadratic equation
    double discriminant = pow(b, 2) - 4 * a * c;

    // Check if the quadratic equation has real solutions
    if (discriminant >= 0) {
        // Calculate the intersection points
        Point intersection1 = {0, 0}, intersection2 = {0, 0};
        double t1 = (-b + sqrt(discriminant)) / (2 * a);
        double t2 = (-b - sqrt(discriminant)) / (2 * a);

        intersection1.x = line_p1.x + t1 * (line_p2.x - line_p1.x);
        intersection1.y = line_p1.y + t1 * (line_p2.y - line_p1.y);
        intersection2.x = line_p1.x + t2 * (line_p2.x - line_p1.x);
        intersection2.y = line_p1.y + t2 * (line_p2.y - line_p1.y);

        // Check if the intersection points are on the line segment
        bool on_segment1 = t1 >= 0 && t1 <= 1;
        bool on_segment2 = t2 >= 0 && t2 <= 1;

        // Check if the distance between either endpoint of the line segment and the circle is greater than 90 cm
        double distance_p1 = sqrt(pow(line_p1.x - center.x, 2) + pow(line_p1.y - center.y, 2));
        double distance_p2 = sqrt(pow(line_p2.x - center.x, 2) + pow(line_p2.y - center.y, 2));

        // Set the first element of the result vector to true if the line segment intersects the circle in two points and the distance between either endpoint of the line segment and the circle is greater than 90 cm
        if (on_segment1 || on_segment2) {
            result[0] = true;
            // Set the second element of the result vector to 1 if distance_p2 > 0.9, otherwise set it to 0
            result[1] = distance_p2 > 0.9 ? 1 : 0;
        }
    }

    return result;
}

extern "C" {

//jintArray JNICALL
jdoubleArray JNICALL
Java_com_example_nativeopencvandroidtemplate_MainActivity_adaptiveThresholdFromJNI(JNIEnv *env,
                                                                                   jobject instance,
                                                                                   jlong matAddr,
                                                                                   jdouble isLine,
                                                                                   jdouble endPoint,
                                                                                   jobjectArray lines


) {

    /////////////////////detection code////////////////////////////
    // get Mat from raw address
    Mat &mat = *(Mat *) matAddr;

    Mat hsv, mask;
//    tmm fi al kolia
    Scalar lower_red(100, 150, 0);
    Scalar upper_red(140, 255, 255);
//tmm  3nd sara
//    Scalar lower_red(33, 103, 143);
//    Scalar upper_red(179, 255, 255);
//
//    Scalar lower_red(0, 111, 186);
//    Scalar upper_red(70, 255, 255);
    vector<vector<Point>> contours;
    cvtColor(mat, hsv, COLOR_BGR2HSV);
    inRange(hsv, lower_red, upper_red, mask);
    erode(mask, mask, getStructuringElement(MORPH_RECT, Size(5, 5)));
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    Point2f center;
    for (size_t i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        vector<Point> approx;
        approxPolyDP(contours[i], approx, 0.02 * arcLength(contours[i], true), true);
        float radius;
        minEnclosingCircle(approx, center, radius);

        if (area > 20) {
            if (approx.size() == 4) {
                drawContours(mat, contours, i, Scalar(255, 255, 255), 5);
                circle(mat, center, 7, Scalar(0, 255, 0), -1);
                __android_log_print(ANDROID_LOG_DEBUG, TAG, "drawing the circle1");
                // putText(mat, "center", Point(center.x - 20, center.y - 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 2);
                //  putText(mat, "Rectangle", approx[0], FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 0));
            }
        }
    }
//    __android_log_print(ANDROID_LOG_DEBUG, TAG, "first center %d %d", center.x,center.y );
    //for checking the car on which line
    int i = 0;
    jsize len = env->GetArrayLength(lines);

    if (isLine == -1) {

        jsize rows = env->GetArrayLength(lines);
        jsize cols = env->GetArrayLength(static_cast<jarray>(env->GetObjectArrayElement(lines, 0)));
//        __android_log_print(ANDROID_LOG_DEBUG, TAG, "Lines size %d  ", rows );

        int count=0;

        for (int i = 0; i < rows; i++) {

            jdoubleArray innerArray = static_cast<jdoubleArray>( env->GetObjectArrayElement(lines, i));
            jdouble *elements = env->GetDoubleArrayElements(innerArray, NULL);
            double l0;
            double l1;
            double l2;
            double l3;
            for (int j = 0; j < cols; j++) {
                jdouble item = elements[j];
                // Do something with item
                if(j==0)
                    l0=item;
                else if(j==1)
                    l1=item;
                else if(j==2)
                    l2=item;
                else
                    l3=item;

            }
//            __android_log_print(ANDROID_LOG_DEBUG, TAG, "2 center %d %d", center.x,center.y );
            double x1 = l0, y1 =l1, x2 = l2, y2 = l3; // The endpoints of the line segment
//            __android_log_print(ANDROID_LOG_DEBUG, TAG, "Line %d %d %d %d ", x1,y1,x2,y2 );

//            __android_log_print(ANDROID_LOG_DEBUG, TAG, "first center %d %d", center.x,center.y );

            // Calculate the length of the line segment
            double length = std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
//            __android_log_print(ANDROID_LOG_DEBUG, TAG, "Line segment length %d ", length );

            // Calculate the direction vector of the line segment
            double dx = (x2 - x1) / length;
            double dy = (y2 - y1) / length;

            // Calculate the vector from the start of the line segment to the center of the circle
            double cx1 = center.x - x1;
            double cy1 = center.y- y1;

            // Calculate the dot product of the direction vector and the vector from the start of the line segment to the center of the circle
            double dotProduct = cx1 * dx + cy1 * dy;

            // Calculate the projection of the center of the circle onto the line
            double projx = x1 + dotProduct * dx;
            double projy = y1 + dotProduct * dy;

            // Calculate the distance between the center of the circle and the projection
            double dist = std::sqrt(std::pow(center.x - projx, 2) + std::pow(center.y - projy, 2));

            // Check if the circle intersects the line segment
            if (dist <= 50 && dotProduct >= 0 && dotProduct <= length) {
                // Calculate the distance between the center and each end point of the line segment
                double dist1 = std::sqrt(std::pow(center.x - x1, 2) + std::pow(center.y - y1, 2));
                double dist2 = std::sqrt(std::pow(center.x - x2, 2) + std::pow(center.y - y2, 2));
//                __android_log_print(ANDROID_LOG_DEBUG, TAG, "distances %d %d", dist1,dist2 );

                // Set the endpoint to the farther point from the center
                if (dist1 >= dist2) {
                    endPoint = 0;
//                    __android_log_print(ANDROID_LOG_DEBUG, TAG, "endPoint1 %d ", endPoint );
                } else {
                    endPoint = 1;
//                    __android_log_print(ANDROID_LOG_DEBUG, TAG, "endPoint2 %d ", endPoint );
                }

                isLine=i;
                count++;
            }
//            __android_log_print(ANDROID_LOG_DEBUG, TAG, "3 center %d %d", center.x,center.y );

            //line( mat, Point(l0, l1), Point(l2, l3), Scalar(0,0,255), 3, LINE_AA);
            env->ReleaseDoubleArrayElements(innerArray, elements, JNI_ABORT);
        }
//        __android_log_print(ANDROID_LOG_DEBUG, TAG, "counttt %d ", count);
    }

    //    for printing the size of the lines
//    __android_log_print(ANDROID_LOG_DEBUG, TAG, "linspafter (%d)", len);
    circle(mat, center, 50, Scalar(0, 255, 255), 6, LINE_AA);

//to draw the circle on the car
    //circle(mat, center, 50, Scalar(0, 255, 255), 6, LINE_AA);
    //    to check the threshold of the endpoint of the line to slow down
    if (isLine != -1) {
        std::vector<cv::Vec4i> lines(len);
        int endx, endy;
        if (endPoint == 0) {
            endx = lines[isLine][0];
            endy = lines[isLine][1];
        } else {
            endx = lines[isLine][2];
            endy = lines[isLine][3];
        }

        if ((sqrt(pow((endx - center.x), 2) + pow((endy - center.y), 2))) < 5) {
            isLine = -1;
            endPoint = -1;
            __android_log_print(ANDROID_LOG_DEBUG, TAG, "Near to the end point");


        }
    }
    jdoubleArray  myArray = env->NewDoubleArray(2); // create a new jintArray with length 3
    jdouble elements[] = {isLine, endPoint}; // initialize an array of jint values
    env->SetDoubleArrayRegion(myArray, 0, 2, elements);

    return myArray;
}

////////////////////////Hough transform///////////////////////

jobjectArray JNICALL Java_com_example_nativeopencvandroidtemplate_MainActivity_getHoughFromJNI(JNIEnv *env, jobject instance, jlong matAddr) {

    // get Mat from raw address
    Mat &mat = *(Mat *) matAddr;

    Mat mat1;
    cvtColor(mat,mat1,COLOR_BGR2GRAY);
    Mat dst;
    // Edge detection
    Canny(mat1, dst, 50, 200, 3);
    // Probabilistic Line Transform
    vector<Vec4i> linesP; // will hold the results of the detection
    HoughLinesP(dst, linesP, 1, CV_PI/180, 50, 200, 50); // runs the actual detection
    vector<Vec2i> mc;

    //  returned matrix
//    jobjectArray matrix = env->NewObjectArray(linesP.size(), env->FindClass("[I"), NULL);
    jobjectArray matrix = env->NewObjectArray(linesP.size(), env->FindClass("[D"), NULL);
    // Draw the lines
//    for( size_t i = 0; i < linesP.size(); i++ ) {
//        Vec4i l = linesP[i];
//        line( mat, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
//        __android_log_print(ANDROID_LOG_DEBUG, TAG, "Line NUM %d Coordinates (%d, %d,%d,%d)", i,l[0], l[1],l[2],l[3]);
//       jintArray arr = env->NewIntArray(4);
//        env->SetIntArrayRegion(arr, 0, 4, linesP[i].val);
//        env->SetObjectArrayElement(matrix, i, arr);
//        env->DeleteLocalRef(arr);
//
//    }


    for (size_t i = 0; i < linesP.size(); i++) {
        Vec4i l = linesP[i];
        line(mat, Point2d((double)l[0], (double)l[1]), Point2d((double)l[2], (double)l[3]), Scalar(0, 0, 255), 3, LINE_AA);
        __android_log_print(ANDROID_LOG_DEBUG, TAG, "Line NUM %d Coordinates (%lf, %lf,%lf,%lf)", i, (double)l[0], (double)l[1], (double)l[2], (double)l[3]);
        jdouble values[4];
        for (int j = 0; j < 4; j++) {
            values[j] = (jdouble)l[j];
        }
        jdoubleArray arr = env->NewDoubleArray(4);
        env->SetDoubleArrayRegion(arr, 0, 4, values);
        env->SetObjectArrayElement(matrix, i, arr);
        env->DeleteLocalRef(arr);
    }


//    jintArray myArray = env->NewIntArray(3); // create a new jintArray with length 3
//    jint elements[] = {1, 2, 3}; // initialize an array of jint values
//    env->SetIntArrayRegion(myArray, 0, 3, elements);
    jdoubleArray myArray = env->NewDoubleArray(3); // create a new jdoubleArray with length 3
    jdouble elements[] = {1.0, 2.0, 3.0}; // initialize an array of jdouble values
    env->SetDoubleArrayRegion(myArray, 0, 3, elements);

    return matrix;
}
//////////////////draw hough lines/////////////////////

void JNICALL Java_com_example_nativeopencvandroidtemplate_MainActivity_drawLinesFromJNI(JNIEnv *env, jobject instance, jlong matAddr, jobjectArray lines) {

    // get Mat from raw address
    Mat &mat = *(Mat *) matAddr;
    jsize rows = env->GetArrayLength(lines);
    jsize cols = env->GetArrayLength(static_cast<jarray>(env->GetObjectArrayElement(lines, 0)));
    vector<Vec2i> mc;


// Iterate over the rows and columns of the matrix
    for (int i = 0; i < rows; i++) {

        jintArray innerArray = static_cast<jintArray>(env->GetObjectArrayElement(lines, i));
        jint *elements = env->GetIntArrayElements(innerArray, NULL);
        int l0;
        int l1;
        int l2;
        int l3;
        for (int j = 0; j < cols; j++) {
            jint item = elements[j];
            // Do something with item
            if(j==0)
                l0=item;
            else if(j==1)
                l1=item;
            else if(j==2)
                l2=item;

            else
                l3=item;
        }
        int m = (l3 - l1) / (l2 - l0);
        int c = l1 - m * l0;
        __android_log_print(ANDROID_LOG_DEBUG, TAG, "Line NUM %d y= %d x + %d ", i,m, c);



        line( mat, Point(l0, l1), Point(l2, l3), Scalar(0,0,255), 3, LINE_AA);

        env->ReleaseIntArrayElements(innerArray, elements, JNI_ABORT);
    }
}
}