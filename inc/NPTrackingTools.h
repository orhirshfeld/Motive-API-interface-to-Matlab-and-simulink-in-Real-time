
//====================================================================================-----
//== Tracking Tools DLL Library
//== Copyright NaturalPoint, Inc.
//==
//== The Rigid Body DLL Library is designed to be a simple yet full featured interface to 
//== the Rigid Body Library.  
//==
//====================================================================================-----

#ifndef NPTRACKINGTOOLS_H
#define NPTRACKINGTOOLS_H

//== Includes ========================================================================-----

//#include "trackablesettings.h"

//== DLL EXPORT/IMPORT PREPROCESSOR DEFINES ==========================================-----

#ifdef NPTRACKINGTOOLS_EXPORTS
#define TTAPI __declspec(dllexport)
#else
#ifndef STATIC_TT_LINK
#define TTAPI __declspec(dllimport)
#else
#define TTAPI  
#endif
#endif

// namespace CameraLibrary
// {
//     class Camera;
//     class CameraManager;
//     class cCameraModule;
// }
// 
// namespace Core
// {
//     struct DistortionModel;
// }

// struct Marker;

//== NPRIGIDBODY PUBLIC INTERFACE ===================================================-----

#define NPRESULT int                  //== NPRESULT Defines Call Success/Failure ====-----

//== RIGID BODY STARTUP / SHUTDOWN ==================================================-----

TTAPI   NPRESULT TT_Initialize();      //== Initialize Library ======================-----
TTAPI   NPRESULT TT_Shutdown();        //== Shutdown Library ========================-----
TTAPI   NPRESULT TT_FinalCleanup();    //== This shuts down device driver, call =====-----
                                       //== this before exiting your application. ===-----

//== RIGID BODY INTERFACE ===========================================================-----

TTAPI   NPRESULT TT_LoadCalibration(const char *filename); //== Load Calibration =====----
TTAPI   NPRESULT TT_LoadTrackables (const char *filename); //== Load Trackables ======----
TTAPI   NPRESULT TT_SaveTrackables (const char *filename); //== Save Trackables ======----
TTAPI   NPRESULT TT_AddTrackables  (const char *filename); //== Add  Trackables ======----
TTAPI   NPRESULT TT_Update();                          //== Process incoming camera data -
TTAPI   NPRESULT TT_UpdateSingleFrame();               //== Process incoming camera data -
TTAPI   NPRESULT TT_LoadProject(const char *filename); //== Load Project File ==========--
TTAPI   NPRESULT TT_SaveProject(const char *filename); //== Save Project File ==========--

// TTAPI   NPRESULT TT_LoadCalibrationFromMemory(unsigned char* Buffer, int BufferSize);

//== RIGID BODY STREAMING ===========================================================-----

TTAPI   NPRESULT TT_StreamTrackd(bool enabled);        //== Start/stop Trackd Stream =----
TTAPI   NPRESULT TT_StreamVRPN(bool enabled, int port);//== Start/stop VRPN Stream ===----
TTAPI   NPRESULT TT_StreamNP    (bool enabled);        //== Start/stop NaturalPoint Stream 

//== FRAME ==========================================================================-----

TTAPI   int      TT_FrameMarkerCount();               //== Returns Frame Markers Count ---
TTAPI   float    TT_FrameMarkerX(int index);          //== Returns X Coord of Marker -----
TTAPI   float    TT_FrameMarkerY(int index);          //== Returns Y Coord of Marker -----
TTAPI   float    TT_FrameMarkerZ(int index);          //== Returns Z Coord of Marker -----
TTAPI   int      TT_FrameMarkerLabel(int index);      //== Returns Label of Marker -------
TTAPI   double   TT_FrameTimeStamp();                 //== Time Stamp of Frame (seconds) -

                 //== TT_FrameCameraCentroid returns true if the camera is contributing
                 //== to this 3D marker.  It also returns the location of the 2D centroid
                 //== that is reconstructing to this 3D marker.

// TTAPI   bool     TT_FrameCameraCentroid(int index, int CameraIndex, float &X, float &Y);

//== TRACKABLES CONTROL =============================================================-----

TTAPI   bool     TT_IsTrackableTracked(int index); //== Is trackable currently tracked ---
TTAPI   void     TT_TrackableLocation(int RigidIndex,       //== Trackable Index ======---
                        float *x, float *y, float *z,                  //== Position ==---
                        float *qx, float *qy, float *qz, float *qw,    //== Orientation -- 
                        float *yaw, float *pitch, float *roll);        //== Orientation --

TTAPI   void     TT_ClearTrackableList();             //== Clear all trackables   =====---
//TTAPI   NPRESULT TT_RemoveTrackable(int Index);       //== Remove single trackable ====---
TTAPI   int      TT_TrackableCount();                 //== Returns number of trackables  -

TTAPI   int      TT_TrackableID(int index);           //== Get Trackables ID ==========---
TTAPI   void     TT_SetTrackableID(int index,int ID); //== Set Trackables ID ==========---
TTAPI   const char* TT_TrackableName(int index);      //== Returns Trackable Name =====---

TTAPI   void     TT_SetTrackableEnabled(int index, bool enabled);    //== Set Tracking   ====---
TTAPI   bool     TT_TrackableEnabled(int index);                     //== Get Tracking   ====---

TTAPI   NPRESULT TT_TrackableTranslatePivot(int index, float x, float y, float z);

TTAPI   int      TT_TrackableMarkerCount(int index);             //== Get marker count   ====---
TTAPI   void     TT_TrackableMarker(int RigidIndex,              //== Get Trackable mrkr ====---
                        int MarkerIndex, float *x, float *y, float *z);
// TTAPI   void     TT_TrackablePointCloudMarker(int RigidIndex,    //== Get corresponding point cloud marker ======---
//                         int MarkerIndex, bool &Tracked,          //== If tracked is false, there is no
//                         float &x, float &y, float &z);           //== corresponding point cloud marker.

                 //== TT_CreateTrackable.  This creates a trackable based on the marker
                 //== count and marker list provided.  The MarkerList is a expected to
                 //== contain of list of marker coordinates in the order: x1,y1,z1,x2,
                 //== y2,z2,etc...xN,yN,zN.

TTAPI   NPRESULT TT_CreateTrackable(const char* Name, int ID, int MarkerCount,
                                    float *MarkerList);

// TTAPI   NPRESULT TT_TrackableSettings   (int Index, cTrackableSettings &Settings);  //== Get Trackable Settings =---
// TTAPI   NPRESULT TT_SetTrackableSettings(int Index, cTrackableSettings &Settings);  //== Set Trackable Settings =---

//== CAMERA MANAGER ACCESS ====================================================================================-----

// TTAPI   CameraLibrary::CameraManager *  TT_GetCameraManager();  //== Returns a pointer to the Camera SDK's  =====---
//                                                                 //== CameraManager ==============================---

//== CAMERA GROUP SUPPORT =====================================================================================-----
// 
// TTAPI   int      TT_CameraGroupCount();               //== Returns number of camera groups ======================---
// TTAPI   bool     TT_CreateCameraGroup();              //== Add an additional group ==============================---
// TTAPI   bool     TT_RemoveCameraGroup(int index);     //== Remove a camera group (must be empty) ================---
// TTAPI   int      TT_CamerasGroup(int index);          //== Returns Camera's camera group index ==================---
// 
// TTAPI   void     TT_SetGroupShutterDelay(int GroupIndex, int Microseconds); //== Set camera group's shutter delay --
// TTAPI   void     TT_SetCameraGroup(int CameraIndex, int CameraGroupIndex);  //== Move camera to camera group ====---
// TTAPI   void     TT_SetGroupInterleaved(int GroupIndex, bool Interleaved);  //== Set camera group's interleaving ---

//== CAMERA GROUP FILTER SETTINGS ===============================================================================---
// 
// class TTAPI cCameraGroupFilterSettings
// {
// public:
//     cCameraGroupFilterSettings();
//     ~cCameraGroupFilterSettings();
// 
//     enum eFilterType
//     {
//         FilterNone,
//         FilterSizeRoundness,
//         FilterCount
//     };
// 
//     eFilterType  FilterType;
//     int          MinMarkerSize;
//     int          MaxMarkerSize;
//     float        MinRoundness;
// };
// 
// TTAPI   NPRESULT TT_CameraGroupFilterSettings   (int GroupIndex, cCameraGroupFilterSettings & Settings);
// TTAPI   NPRESULT TT_SetCameraGroupFilterSettings(int GroupIndex, cCameraGroupFilterSettings & Settings);
// 
// TTAPI   NPRESULT TT_SetEnabledFilterSwitch(bool Enabled);  //== Enabled by default ==--
// TTAPI   bool     TT_IsFilterSwitchEnabled();
// 
// class TTAPI cCameraGroupMarkerSizeSettings
// {
// public:
//     cCameraGroupMarkerSizeSettings();
//     ~cCameraGroupMarkerSizeSettings();
// 
//     enum eMarkerSizeType
//     {
//         MarkerSizeCalculated,
//         MarkerSizeFixed,
//         MarkerSizeCount
//     };
// 
//     eMarkerSizeType  MarkerSizeType;
//     float            MarkerSize;
// };
// 
// TTAPI   NPRESULT TT_CameraGroupMarkerSize   (int GroupIndex, cCameraGroupMarkerSizeSettings & Settings);
// TTAPI   NPRESULT TT_SetCameraGroupMarkerSize(int GroupIndex, cCameraGroupMarkerSizeSettings & Settings);
// 
// TTAPI   NPRESULT TT_SetCameraGroupReconstruction(int GroupIndex, bool Enable);
// TTAPI   NPRESULT TT_SetCameraGroupInterleaved(int GroupIndex, bool Enable);

//== POINT CLOUD INTERFACE ==========================================================-----

TTAPI   int      TT_CameraCount();                    //== Returns Camera Count =====-----
TTAPI   float    TT_CameraXLocation(int index);       //== Returns Camera's X Coord =-----
TTAPI   float    TT_CameraYLocation(int index);       //== Returns Camera's Y Coord =-----
TTAPI   float    TT_CameraZLocation(int index);       //== Returns Camera's Z Coord =-----
TTAPI   float    TT_CameraOrientationMatrix(int camera, int index); //== Orientation -----

TTAPI   const char* TT_CameraName(int index);         //== Returns Camera Name ======-----

TTAPI   int      TT_CameraMarkerCount(int CameraIndex); //== Camera's 2D Marker Count =---

                 //== CameraMarker fetches the 2D centroid location of the marker as seen
                 //== by the camera.

// TTAPI   bool     TT_CameraMarker(int CameraIndex, int MarkerIndex, float &x, float &y);

                 //== Fetch predistorted marker location.  This is basically where the
                 //== camera would see the marker if there was no lens distortion.
                 //== For most of our cameras/lenses, this location is only a few pixels
                 //== from the distorted (TT_CameraMarker) position.

// TTAPI   bool     TT_CameraMarkerPredistorted(int CameraIndex, int MarkerIndex, float &x, float &y);

                 //== Set camera settings.  This function allows you to set the camera's
                 //== video mode, exposure, threshold, and illumination settings.
                 
                 //== VideoType:  
                 //==     0 = Segment Mode   
                 //==     1 = Grayscale Mode 
                 //==     2 = Object Mode    
                 //==     4 = Precision Mode
                 //==     6 = MJPEG Mode     (V100R2 only)

                 //== Exposure: Valid values are:  1-480
                 //== Threshold: Valid values are: 0-255
                 //== Intensity: Valid values are: 0-15  (This should be set to 15 for all most all
                 //==                                     situations)
                  
TTAPI   bool     TT_SetCameraSettings(int CameraIndex, int VideoType, int Exposure, int Threshold, int Intensity);

// TTAPI   int      TT_CameraGrayscaleDecimation(int CameraIndex); //== Camera's Full Frame Grayscale Decimation =---
// TTAPI   bool     TT_SetCameraGrayscaleDecimation(int CameraIndex, int Value);
// 
//                  //== Toggle camera extended options
// 
// TTAPI   bool     TT_SetCameraFilterSwitch(int CameraIndex, bool EnableIRFilter);
// TTAPI   bool     TT_SetCameraAGC(int CameraIndex, bool EnableAutomaticGainControl);
// TTAPI   bool     TT_SetCameraAEC(int CameraIndex, bool EnableAutomaticExposureControl);
// TTAPI   bool     TT_SetCameraHighPower(int CameraIndex, bool EnableHighPowerMode);
// TTAPI   bool     TT_SetCameraMJPEGHighQuality(int CameraIndex, int MJPEGQuality);


                 //== Fetch the cameras frame buffer.  This function fills the provided
                 //== buffer with an image of what is in the camera view.  The resulting
                 //== Image depends on what video mode the camera is in.  If the camera
                 //== is in grayscale mode, for example, a grayscale image is returned
                 //== from this call.

// TTAPI   bool     TT_CameraFrameBuffer(int CameraIndex, int BufferPixelWidth, int BufferPixelHeight,
//                                       int BufferByteSpan, int BufferPixelBitDepth, unsigned char *Buffer);
// 
//                  //== Save camera's frame buffer as a BMP file
// 
// TTAPI   bool     TT_CameraFrameBufferSaveAsBMP(int CameraIndex, const char *Filename);

                 //== Backproject from 3D space to 2D space.  If you give this function a 3D
                 //== location and select a camera, it will return where the point would land
                 //== on the imager of that camera in to 2D space.  This basically locates
                 //== where in the cameras FOV a 3D point would be located.

// TTAPI   void     TT_CameraBackproject(int CameraIndex, float X, float Y, float Z, float &CameraX, float &CameraY);
// 
//                  //== The 2D centroids the camera reports are distorted by the lens.  To remove
//                  //== the distortion call CameraUndistort2DPoint.  Also if you have a 2D undistorted point
//                  //== that you'd like to convert back to a distorted point call CameraDistort2DPoint.
// 
// TTAPI   void     TT_CameraUndistort2DPoint(int CameraIndex, float &X, float &Y);
// TTAPI   void     TT_CameraDistort2DPoint  (int CameraIndex, float &X, float &Y);

                 //== TT_CameraRay will take an undistorted 2D centroid and return a camera ray in the world
                 //== coordinate system.

// TTAPI   bool     TT_CameraRay(int CameraIndex, float X, float Y, float &RayStartX, float &RayStartY, float &RayStartZ,
//                                                                  float &RayEndX,   float &RayEndY,   float &RayEndZ);
// 
//                  //== Set camera's extrinsic (position & orientation) and intrinsic (lens distortion) parameters with
//                  //== parameters compatible with the OpenCV intrinsic model.
// 
// TTAPI   bool     TT_CameraModel(int CameraIndex, float X, float Y, float Z, //== Camera Position ===============----
//                                 float *Orientation,                         //== Orientation (3x3 matrix) ======----
//                                 float PrincipleX, float PrincipleY,         //== Lens center (in pixels)  ======----
//                                 float FocalLengthX, float FocalLengthY,     //== Lens focal  (in pixels)  ======----
//                                 float KC1, float KC2, float KC3,            //== Barrel distortion coefficients ----
//                                 float Tangential0, float Tangential1);      //== Tangential distortion =========----

//== Additional Functionality =================================================================================-----

//== This function will return the Camera SDK's camera pointer.  While the Tracking Tools API takes over the
//== data path which prohibits fetching the frames directly from the camera, it is still very usefully to be
//== able to communicate with the camera directly for setting camera settings or attaching modules.

// TTAPI   CameraLibrary::Camera * TT_GetCamera(int index);  //== Return's Camera SDK Camera
// 
// TTAPI   void TT_AttachCameraModule(int Index, CameraLibrary::cCameraModule *Module);
// TTAPI   void TT_DetachCameraModule(int Index, CameraLibrary::cCameraModule *Module);

//== Rigid Body Solver Callback Hook ==========================================================================-----

//== Inherit cTrackableSolutionTest and overload the TrackableSolutionTest method to have the ability to reject
//== potential rigid body solutions during the rigid body solving process.  You must attach your cTrackableSolutionTest
//== class to a rigid body via TT_AttachTrackableSolutionTest.  Return false if the presented solution should be
//== rejected.

// class TTAPI cTrackableSolutionTest
// {
// public:
//     cTrackableSolutionTest()  {};
//     ~cTrackableSolutionTest() {};
// 
//     //== Trackable Solution Test ==--
// 
//     virtual bool TrackableSolutionTest(int MarkerCount, Marker *Markers, bool *MarkerExists) { return true; };
// };
// 
// TTAPI   void     TT_AttachTrackableSolutionTest(int index, cTrackableSolutionTest* Test);
// TTAPI   void     TT_DetachTrackableSolutionTest(int index, cTrackableSolutionTest* Test);
// 
// //== Tracking Tools API Callbacks =============================================================================-----
// 
// //== Inherit cTTAPIListener and override it's methods to receive callbacks from the TTAPI.  You must attach your
// //== listening class to the TTAPI via TT_AttachListener.
// 
// class TTAPI cTTAPIListener
// {
// public:
//     cTTAPIListener()  {};
//     ~cTTAPIListener() {};
// 
//     //== TTAPIFrameAvailable callback is called when a new synchronized group of camera frames has been delivered
//     //== to the TTAPI and is ready for processing.  You can use this notification to then call TT_Update() without
//     //== having to poll blindly for new data.
// 
//     virtual void TTAPIFrameAvailable() {};
// 
//     //== InitialPointCloud is called when the initial point cloud is calculated from the connected cameras. During
//     //== this callback 3D markers can be added (up to MaxMarkers) or removed by modifying the Markers list as well
//     //== as the MarkerCount variable.  After this callback the marker list is passed onto the rigid body solver.
// 
//     virtual void InitialPointCloud(Marker* Markers, int &MarkerCount, int MaxMarkers) {};
// };
// 
// TTAPI   void     TT_AttachListener(cTTAPIListener* Listener);
// TTAPI   void     TT_DetachListener(cTTAPIListener* Listener);
// 
// //== RESULT PROCESSING ========================================================================================-----

TTAPI   const char *TT_GetResultString(NPRESULT result); //== Return Plain Text Message =======================-----

#define NPRESULT_SUCCESS                0             //== Successful Result ================================-------
#define NPRESULT_FILENOTFOUND           1             //== File Not Found ===================================-------
#define NPRESULT_LOADFAILED             2             //== Load Failed ======================================-------
#define NPRESULT_FAILED                 3             //== Failed ===========================================-------
#define NPRESULT_INVALIDFILE            8             //== Invalid File =====================================-------
#define NPRESULT_INVALIDCALFILE         9             //== Invalid Calibration File =========================-------
#define NPRESULT_UNABLETOINITIALIZE     10            //== Unable To Initialize =============================-------
#define NPRESULT_INVALIDLICENSE         11            //== Invalid License ==================================-------
#define NPRESULT_NOFRAMEAVAILABLE       14            //== No Frames Available ==============================-------

//== CAMERA VIDEO TYPE DEFINITIONS ==========================================================================-------

#define NPVIDEOTYPE_SEGMENT   0
#define NPVIDEOTYPE_GRAYSCALE 1
#define NPVIDEOTYPE_OBJECT    2
#define NPVIDEOTYPE_PRECISION 4
#define NPVIDEOTYPE_MJPEG     6

//===============================================================================================================---

#endif