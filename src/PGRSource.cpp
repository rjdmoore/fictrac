/// FicTrac http://rjdmoore.net/fictrac/
/// \file       PGRSource.cpp
/// \brief      PGR USB3 sources (Spinnaker SDK).
/// \author     Richard Moore
/// \copyright  CC BY-NC-SA 3.0

#ifdef PGR_USB3

#include "PGRSource.h"

#include "Logger.h"

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
            LOG_DBG("Found %d PGR cameras.", numCameras);
        }

        // Select camera
        _cam = _camList.GetByIndex(index);

        // Initialize camera
        _cam->Init();

        // Begin acquiring images
        _cam->BeginAcquisition();

        // Get some params
        _width = _cam->Width();
        _height = _cam->Height();
        _fps = _cam->AcquisitionFrameRate();

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

double PGRSource::getFPS()
{
    // do nothing
    return _fps;
}

bool PGRSource::setFPS(double fps)
{
    _fps = fps;
    return false;
}

bool PGRSource::grab(cv::Mat& frame)
{
	if( !_open ) { return NULL; }

    ImagePtr pgr_image = NULL;

    try {
        // Retrieve next received image
        long int timeout = _fps > 0 ? std::max(static_cast<long int>(1000), static_cast<long int>(1000. / _fps)) : 1000; // set capture timeout to at least 1000 ms
        pgr_image = _cam->GetNextImage(timeout);
        _timestamp = _cam->Timestamp();

        // Ensure image completion
        if (pgr_image->IsIncomplete()) {
            // Retreive and print the image status description
            LOG_ERR("Error! Image capture incomplete (%s).", Image::GetImageStatusDescription(pgr_image->GetImageStatus()));
            pgr_image->Release();
            return false;
        }

        // Convert image
        ImagePtr bgr_image = pgr_image->Convert(PixelFormat_BGR8, NEAREST_NEIGHBOR);

        // We have to release our original image to clear space on the buffer
        pgr_image->Release();

        Mat tmp(_height, _width, CV_8UC3, bgr_image->GetData(), bgr_image->GetStride());
        tmp.copyTo(frame);

        return true;
    }
    catch (Spinnaker::Exception& e) {
        LOG_ERR("Error grabbing PGR frame! Error was: %s", e.what());
        pgr_image->Release();
        return false;
    }
}

PGRSource::~PGRSource()
{
	if( _open ) {
        _cam->EndAcquisition();
        _open = false;
	}
    _cam = NULL;
	
    // Clear camera list before releasing system
    _camList.Clear();

    // Release system
    _system->ReleaseInstance();
}

#endif // PGR_USB3
