/// FicTrac http://rjdmoore.net/fictrac/
/// \file       PGRSource.cpp
/// \brief      PGR USB3 sources (Spinnaker SDK).
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#ifdef PGR_USB3

#include "PGRSource.h"

#include "Logger.h"
#include "timing.h"

#include "SpinGenApi/SpinnakerGenApi.h"

using namespace Spinnaker;
using cv::Mat;

PGRSource::PGRSource(int index)
{
    try {
        // Retrieve singleton reference to system object
        _system = System::GetInstance();

        // Print out current library version
        const LibraryVersion spinnakerLibraryVersion = _system->GetLibraryVersion();
        LOG("Opening PGR camera using Spinnaker SDK (version %d.%d.%d.%d)",
            spinnakerLibraryVersion.major, spinnakerLibraryVersion.minor,
            spinnakerLibraryVersion.type, spinnakerLibraryVersion.build);

        // Retrieve list of cameras from the system
        _camList = _system->GetCameras();

        unsigned int numCameras = _camList.GetSize();

        if (numCameras == 0) {
            LOG_ERR("Error! Could not find any connected PGR cameras!");
            return;
        }
        else {
            LOG_DBG("Found %d PGR cameras. Connecting to camera %d..", numCameras, index);
        }

        // Select camera
        _cam = _camList.GetByIndex(index);

        // Initialize camera
        _cam->Init();

        // set acquisition mode - needed?
        {
            // Retrieve GenICam nodemap
            Spinnaker::GenApi::INodeMap& nodeMap = _cam->GetNodeMap();

            // Retrieve enumeration node from nodemap
            Spinnaker::GenApi::CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
            if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode)) {
                LOG_ERR("Unable to set acquisition mode to continuous (enum retrieval)!");
                return;
            }

            // Retrieve entry node from enumeration node
            Spinnaker::GenApi::CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
            if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous)) {
                LOG_ERR("Unable to set acquisition mode to continuous (entry retrieval)!");
                return;
            }

            // Retrieve integer value from entry node
            int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

            // Set integer value from entry node as new value of enumeration node
            ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

            LOG_DBG("Acquisition mode set to continuous.");
        }

        // Begin acquiring images
        _cam->BeginAcquisition();

        // Get some params
        _width = _cam->Width();
        _height = _cam->Height();
        _fps = getFPS();

        LOG("PGR camera initialised (%dx%d @ %.3f fps)!", _width, _height, _fps);

        _open = true;
        _live = true;
    }
    catch (Spinnaker::Exception& e) {
        LOG_ERR("Error opening capture device! Error was: %s", e.what());
    }
    catch (...) {
        LOG_ERR("Error opening capture device!");
    }
}

PGRSource::~PGRSource()
{
    if (_open) {
        try {
            _cam->EndAcquisition();
        }
        catch (Spinnaker::Exception& e) {
            LOG_ERR("Error ending acquisition! Error was: %s", e.what());
        }
        catch (...) {
            LOG_ERR("Error ending acquisition!");
        }
        _open = false;
    }
    _cam = NULL;

    // Clear camera list before releasing system
    _camList.Clear();

    // Release system
    _system->ReleaseInstance();
}

double PGRSource::getFPS()
{
    double fps = _fps;
    if (_open) {
        try {
            fps = _cam->AcquisitionResultingFrameRate();
        }
        catch (Spinnaker::Exception& e) {
            LOG_ERR("Error retrieving camera frame rate! Error was: %s", e.what());
        }
        catch (...) {
            LOG_ERR("Error retrieving camera frame rate!");
        }
    }
    return fps;
}

bool PGRSource::setFPS(double fps)
{
    bool ret = false;
    if (_open && (fps > 0)) {
        try {
            _cam->AcquisitionFrameRateEnable.SetValue(true);
            _cam->AcquisitionFrameRate.SetValue(fps);
        }
        catch (Spinnaker::Exception& e) {
            LOG_ERR("Error setting frame rate! Error was: %s", e.what());
        }
        catch (...) {
            LOG_ERR("Error setting frame rate!");
        }
        _fps = getFPS();
        LOG("Device frame rate is now %.2f", _fps);
        ret = true;
    }
    return ret;
}

bool PGRSource::grab(cv::Mat& frame)
{
	if( !_open ) { return NULL; }

    ImagePtr pgr_image = NULL;

    try {
        // Retrieve next received image
        long int timeout = _fps > 0 ? std::max(static_cast<long int>(1000), static_cast<long int>(1000. / _fps)) : 1000; // set capture timeout to at least 1000 ms
        pgr_image = _cam->GetNextImage(timeout);
        double ts = static_cast<double>(ts_ms());    // backup, in case the device timestamp is junk
        _timestamp = _cam->Timestamp();
        if (_timestamp <= 0) {
            _timestamp = ts;
        }

        // Ensure image completion
        if (pgr_image->IsIncomplete()) {
            // Retreive and print the image status description
            LOG_ERR("Error! Image capture incomplete (%s).", Image::GetImageStatusDescription(pgr_image->GetImageStatus()));
            pgr_image->Release();
            return false;
        }
    }
    catch (Spinnaker::Exception& e) {
        LOG_ERR("Error grabbing PGR frame! Error was: %s", e.what());
        pgr_image->Release();
        return false;
    }
    catch (...) {
        LOG_ERR("Error grabbing PGR frame!");
        pgr_image->Release();
        return false;
    }

    try {
        // Convert image
        ImagePtr bgr_image = pgr_image->Convert(PixelFormat_BGR8, NEAREST_NEIGHBOR);

        Mat tmp(_height, _width, CV_8UC3, bgr_image->GetData(), bgr_image->GetStride());
        tmp.copyTo(frame);

        // We have to release our original image to clear space on the buffer
        pgr_image->Release();

        return true;
    }
    catch (Spinnaker::Exception& e) {
        LOG_ERR("Error converting PGR frame! Error was: %s", e.what());
        pgr_image->Release();
        return false;
    }
    catch (...) {
        LOG_ERR("Error converting PGR frame!");
        pgr_image->Release();
        return false;
    }
}

#endif // PGR_USB3
