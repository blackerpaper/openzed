#include "libuvc/libuvc.h"
#include <stdio.h>
#include <unistd.h>

/* This callback function runs once per frame. Use it to perform any
 * quick processing you need, or have it put the frame into your application's
 * input queue. If this function takes too long, you'll start losing frames. */
void cb(uvc_frame_t *frame, void *ptr) {
    uvc_frame_t *bgr;
    uvc_error_t ret;

    /* We'll convert the image from YUV/JPEG to BGR, so allocate space */
    bgr = uvc_allocate_frame(frame->width * frame->height * 3);
    if (!bgr) {
        printf("unable to allocate bgr frame!");
        return;
    }

    /* Do the BGR conversion */
    ret = uvc_any2bgr(frame, bgr);
    if (ret) {
        uvc_perror(ret, "uvc_any2bgr");
        uvc_free_frame(bgr);
        return;
    }

    /* Call a user function:
     *
     * my_type *my_obj = (*my_type) ptr;
     * my_user_function(ptr, bgr);
     * my_other_function(ptr, bgr->data, bgr->width, bgr->height);
     */

    /* Call a C++ method:
     *
     * my_type *my_obj = (*my_type) ptr;
     * my_obj->my_func(bgr);
     */

    /* Use opencv.highgui to display the image:
     *
     * cvImg = cvCreateImageHeader(
     *     cvSize(bgr->width, bgr->height),
     *     IPL_DEPTH_8U,
     *     3);
     *
     * cvSetData(cvImg, bgr->data, bgr->width * 3);
     *
     * cvNamedWindow("Test", CV_WINDOW_AUTOSIZE);
     * cvShowImage("Test", cvImg);
     * cvWaitKey(10);
     *
     * cvReleaseImageHeader(&cvImg);
     */

    uvc_free_frame(bgr);
}

int main(int argc, char **argv) {
    uvc_context_t *ctx;
    uvc_device_t *dev;
    uvc_device_handle_t *devh;
    uvc_stream_ctrl_t ctrl;
    uvc_error_t res;

    /* Initialize a UVC service context. Libuvc will set up its own libusb
     * context. Replace NULL with a libusb_context pointer to run libuvc
     * from an existing libusb context. */
    res = uvc_init(&ctx, NULL);

    if (res < 0) {
        uvc_perror(res, "uvc_init");
        return res;
    }

    puts("UVC initialized");

    /* Locates the first attached UVC device, stores in dev */
    res = uvc_find_device(
            ctx, &dev,
            0, 0, NULL); /* filter devices: vendor_id, product_id, "serial_num" */

    if (res < 0) {
        uvc_perror(res, "uvc_find_device"); /* no devices found */
    } else {
        puts("Device found");

        /* Try to open the device: requires exclusive access */
        res = uvc_open(dev, &devh);

        if (res < 0) {
            uvc_perror(res, "uvc_open"); /* unable to open device */
        } else {
            puts("Device opened");

            /* Print out a message containing all the information that libuvc
             * knows about the device */
            uvc_print_diag(devh, stderr);

            /* Try to negotiate a 640x480 30 fps YUYV stream profile */
            res = uvc_get_stream_ctrl_format_size(
                    devh, &ctrl, /* result stored in ctrl */
                    UVC_FRAME_FORMAT_YUYV, /* YUV 422, aka YUV 4:2:2. try _COMPRESSED */
                    640, 480, 30 /* width, height, fps */
            );

            /* Print out the result */
            uvc_print_stream_ctrl(&ctrl, stderr);

            if (res < 0) {
                uvc_perror(res, "get_mode"); /* device doesn't provide a matching stream */
            } else {
                /* Start the video stream. The library will call user function cb:
                 *   cb(frame, (void*) 12345)
                 */
                res = uvc_start_streaming(devh, &ctrl, cb, (void*)12345, 0);

                if (res < 0) {
                    uvc_perror(res, "start_streaming"); /* unable to start stream */
                } else {
                    puts("Streaming...");

                    uvc_set_ae_mode(devh, 1); /* e.g., turn on auto exposure */

                    sleep(10); /* stream for 10 seconds */

                    /* End the stream. Blocks until last callback is serviced */
                    uvc_stop_streaming(devh);
                    puts("Done streaming.");
                }
            }

            /* Release our handle on the device */
            uvc_close(devh);
            puts("Device closed");
        }

        /* Release the device descriptor */
        uvc_unref_device(dev);
    }

    /* Close the UVC context. This closes and cleans up any existing device handles,
     * and it closes the libusb context if one was not provided. */
    uvc_exit(ctx);
    puts("UVC exited");

    return 0;
}




//#include <iostream>
//#include <usb.h>
//
//using namespace std;
//
//int main() {
//    int ret;
//
//    usb_init();
//    ret = usb_find_busses();
//    ret = usb_find_devices();
//
//    usb_dev_handle *udev;
//    struct usb_bus *bus;
//    struct usb_device *dev;
//    for (bus=usb_get_busses(); bus; bus=bus->next) {
//        for (dev=bus->devices; dev; dev=dev->next) {
//            if(dev->descriptor.idVendor == 11011 && dev->descriptor.idProduct == 62848) {
//                std::cout << "Zed Camera Found!" << std::endl;
//                udev = usb_open(dev);
//
//                break;
//            }
//        }
//    }
//
//    return 0;
//}

//#include "opencv2/opencv.hpp"
//
//using namespace cv;
//
//int main(int, char**)
//{
//    VideoCapture cap(0);
//    if(!cap.isOpened())
//        return -1;
//
//    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
//    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
//
//    Ptr<StereoBM> bm = StereoBM::create(16, 15);
//    Mat mDisparity;
//    for(;;)
//    {
//        Mat frame;
//        cap >> frame;
//
//        int w = frame.cols / 2;
//        int h = frame.rows;
//
//        cv::Rect lRect (0, 0, w, h);
//        cv::Rect rRect (w, 0, w, h);
//
//        cv::Mat left = frame(lRect);
//        cv::Mat right = frame(rRect);
//
//        if( left.channels() > 1  )
//            cv::cvtColor( left, left, CV_BGR2GRAY );
//
//        if( right.channels() > 1  )
//            cv::cvtColor( right, right, CV_BGR2GRAY );
//
//        bm->compute(left, right, mDisparity);
//        normalize(mDisparity, mDisparity, 0, 255, CV_MINMAX, CV_8U);
//
//        imshow("left", left);
//        imshow("right", right);
//        imshow("disparity", mDisparity);
//        if(waitKey(30) >= 0) break;
//    }
//
//    return 0;
//}



//#include "opencv2/opencv.hpp"
//
//using namespace cv;
//
//int main(int, char**)
//{
//    VideoCapture cap(0);
//    if(!cap.isOpened())
//        return -1;
//
//    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
//    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
//
//    Ptr<StereoSGBM> sgbm;
//    sgbm = StereoSGBM::create(-64, 192, 5, 600, 2400, 10, 4, 1, 150, 2, StereoSGBM::MODE_SGBM);
//
//    cv::Mat mDisparity;
//    for(;;)
//    {
//        Mat frame;
//        cap >> frame;
//
//        int w = frame.cols / 2;
//        int h = frame.rows;
//
//        cv::Rect lRect (0, 0, w, h);
//        cv::Rect rRect (w, 0, w, h);
//
//        cv::Mat left = frame(lRect);
//        cv::Mat right = frame(rRect);
//
//        if( left.channels() > 1  )
//            cv::cvtColor( left, left, CV_BGR2GRAY );
//
//        if( right.channels() > 1  )
//            cv::cvtColor( right, right, CV_BGR2GRAY );
//
//        sgbm->compute(left, right, mDisparity);
//        normalize(mDisparity, mDisparity, 0, 255, CV_MINMAX, CV_8U);
//
//        imshow("left", left);
//        imshow("right", right);
//        imshow("disparity", mDisparity);
//        if(waitKey(30) >= 0) break;
//    }
//
//    return 0;
//}


