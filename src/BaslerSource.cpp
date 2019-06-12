/// FicTrac http://rjdmoore.net/fictrac/
/// \file       BaslerSource.cpp
/// \brief      Basler USB3 sources (Pylon SDK).
/// \author     Wenbin Yang, Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#if defined(BASLER_USB3)

#include "BaslerSource.h"

#include "Logger.h"
#include "timing.h"

#include <algorithm>

using namespace std;
using namespace cv;
using namespace Pylon;

BaslerSource::BaslerSource(int index)
{
    try {
        PylonInitialize();
        // create an instant camera object
        _cam.Attach(CTlFactory::GetInstance().CreateFirstDevice());

        LOG("Opening Basler camera device: %s", _cam.GetDeviceInfo().GetModelName());

        // Allow all the names in the namespace GenApi to be used without qualification.
        using namespace GenApi;

        // Get the camera control object.
        INodeMap &control = _cam.GetNodeMap();

        // Start acquisition
        _cam.StartGrabbing();

        // Get some params
        const CIntegerPtr camWidth = control.GetNode("Width");
        const CIntegerPtr camHeight = control.GetNode("Height");

        _width = camWidth->GetValue();
        _height = camHeight->GetValue();
        _fps = getFPS();

        LOG("Basler camera initialised (%dx%d @ %.3f fps)!", _width, _height, _fps);


        _open = true;
        _live = true;

        }
        catch (const GenericException &e) {
        LOG_ERR("Error opening capture device! Error was: %s", e.GetDescription());
    }
}

BaslerSource::~BaslerSource()
{
    if (_open) {
        try {
	        _cam.DetachDevice();
        }
        catch (const GenericException &e) {
            LOG_ERR("Error opening capture device! Error was: %s", e.GetDescription());
        }
        _open = false;
    }
}

double BaslerSource::getFPS()
{
    const GenApi::CFloatPtr camFrameRate = _cam.GetNodeMap().GetNode("ResultingFrameRateAbs");// TODO: test this line
    return camFrameRate->GetValue();
}

bool BaslerSource::setFPS(double fps)
{
    using namespace GenApi;

    bool ret = false;
    if (_open && (fps > 0)) {
        // Get the camera control object.
        INodeMap &control = _cam.GetNodeMap();
        const GenApi::CFloatPtr camFrameRate = _cam.GetNodeMap().GetNode("ResultingFrameRateAbs");// TODO: test this line
        if (IsWritable(camFrameRate))
        {
            camFrameRate->SetValue(fps);
            ret = true;
        }
        else {
            LOG_ERR("Error setting frame rate!");
        }
        _fps = getFPS();
        LOG("Device frame rate is now %.2f", _fps);
    }
}

bool BaslerSource::grab(cv::Mat& frame)
{
    if (!_open) { return false; }

    // Set grab timeout
    long int timeout = _fps > 0 ? max(static_cast<long int>(1000), static_cast<long int>(1000. / _fps)) : 1000; // set capture timeout to at least 1000 ms
    try {
        _cam.RetrieveResult(timeout, _ptrGrabResult, TimeoutHandling_ThrowException);
        double ts = ts_ms();    // backup, in case the device timestamp is junk
        //_timestamp = pgr_image->GetTimeStamp();   // TODO: extract timestamp
        if (!_ptrGrabResult->GrabSucceeded()) {
            LOG_ERR("Error! Image capture failed (%d: %s).", _ptrGrabResult->GetErrorCode(), _ptrGrabResult->GetErrorDescription().c_str());
            // release the original image pointer
            _ptrGrabResult.Release();
            return false;
        }
        else { // TODO: no getNumChannels method found, use GetPixelType() instead.
            LOG_DBG("Frame captured %dx%dx%d @ %f (%f)", _ptrGrabResult->GetWidth(), _ptrGrabResult->GetHeight(), _ptrGrabResult->GetPixelType(), _timestamp, ts);
        }
        if (_timestamp <= 0) {
            _timestamp = ts;
        }
    }
    catch (const GenericException &e) {
        LOG_ERR("Error grabbing frame! Error was: %s", e.GetDescription());
        // release the original image pointer
        _ptrGrabResult.Release();
        return false;
    }

    try {
        // Convert image
        Pylon::CImageFormatConverter formatConverter;
        formatConverter.Convert(_pylonImg, _ptrGrabResult);

        Mat tmp(_height, _width, CV_8UC3, (uint8_t*)_pylonImg.GetBuffer()); 
        tmp.copyTo(frame);

        // release the original image pointer
        _ptrGrabResult.Release();
    }
    catch (const GenericException &e) {
        LOG_ERR("Error converting frame! Error was: %s", e.GetDescription());
        // release the original image pointer
        _ptrGrabResult.Release();
        return false;
    }

    return true;
}

#endif // BASLER_USB3
