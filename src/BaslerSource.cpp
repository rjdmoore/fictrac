
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
  	cam.Attach(CTlFactory::GetInstance().CreateFirstDevice());

    LOG("Opening Basler camera device: %s", cam.GetDeviceInfo().GetModelName());

  	// Allow all the names in the namespace GenApi to be used without qualification.
  	using namespace GenApi;

    // Get the camera control object.
	  INodeMap &control = cam.GetNodeMap();

    // Get some params
    const CIntegerPtr camWidth = control.GetNode("Width");
  	const CIntegerPtr camHeight = control.GetNode("Height");
  	const CIntegerPtr camFrameRate = control.GetNode("FrameRate");// TODO: test this line
    // TODO: assign int values to _width, _height, and _fps

    // Start acquisition
    cam.StartGrabbing();

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
		cam.DetachDevice();
    }
    catch (const GenericException &e) {
      LOG_ERR("Error opening capture device! Error was: %s", e.GetDescription());
    }
    _open = false;
  }
}

double BaslerSource::getFPS()
{
  return _fps;
}

bool BaslerSource::setFPS(double fps)
{
  using namespace GenApi;

  // Get the camera control object.
  INodeMap &control = cam.GetNodeMap();
  const CIntegerPtr camFrameRate = control.GetNode("FrameRate");// TODO: test this line
  if (IsWritable(camFrameRate))
  {
    camFrameRate->SetValue(fps);
    return true;
  }
  else {
    return false;
  }

}


bool BaslerSource::grab(cv::Mat& frame)
{
    if (!_open) { return false; }

  // Set grab timeout

  long int timeout = _fps > 0 ? max(static_cast<long int>(1000), static_cast<long int>(1000. / _fps)) : 1000; // set capture timeout to at least 1000 ms
  try {
    cam.RetrieveResult(timeout, ptrGrabResult, TimeoutHandling_ThrowException);
    if (!ptrGrabResult->GrabSucceeded())
			cout << "Error: " << ptrGrabResult->GetErrorCode() <<
      " " << ptrGrabResult->GetErrorDescription() << endl;
    else // TODO: no getNumChannels method found, use GetPixelType() instead.
      LOG_DBG("Frame captured! %dx%d%s", _width, _height, ptrGrabResult->GetPixelType());
  }
  catch (const GenericException &e) {
    cerr << "An exception occurred." << endl
			<< e.GetDescription() << endl;
  }


  try {
    // Convert image
    Pylon::CImageFormatConverter formatConverter;
  	formatConverter.Convert(pylonImg, ptrGrabResult);

    Mat tmp(_height, _width, CV_8UC3, (uint8_t*)pylonImg.GetBuffer()); 
    tmp.copyTo(frame);

    // release the original image pointer
    ptrGrabResult.Release();
  }
  catch (const GenericException &e) {
    LOG_ERR("Error converting PGR frame! Error was: %s", e.GetDescription());
    return false;
  }



}
