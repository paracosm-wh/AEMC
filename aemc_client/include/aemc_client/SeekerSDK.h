#ifndef SeekerSDK_H
#define SeekerSDK_H

#ifdef __cplusplus

extern "C" {

#endif

# define Seeker_DLL


/** Return codes
*/
typedef enum maReturnCode
{
    RC_Okay = 0,             //!< Okay
    RC_GeneralError,       //!< General Error
    RC_ApiError,           //!< Invalid use of the API
    RC_NetworkError,       //!< Network Error
    RC_TimeOut,            //!< No response from Seeker
    RC_MemoryError,        //!< Memory allocation failed
    RC_Unrecognized        //!< Request string not recognized
}
        maReturnCode;


/** Verbosity setting for internal messages
*/
typedef enum maVerbosityLevel
{
    VL_None = 0,   //!< No Messages
    VL_Error,    //!< Error Message
    VL_Warning,  //!< Warning Message [DEFAULT VALUE]
    VL_Info,     //!< Informational Message
    VL_Debug,    //!< Debug Message
}
        maVerbosityLevel;


// Array dimensions

#define MAX_N_BODIES      100


#define XEMPTY 9999999.0f


/** Data for one segment
*/
typedef double tSegmentData[7]; //!<  X,Y,Z, aX,aY,aZ, Length

/** Data for one marker
*/
typedef float  tMarkerData[3];  //!<  X,Y,Z

/** Data for one forceplate
*/
typedef float  tForceData[7];   //!<  X,Y,Z, fX,fY,fZ, mZ

/** Data for one degree of freedom
*/
typedef double tDofData;        //!<  Usually an angle value in degrees





//! The description of the connection to Seeker.
/*!
This contains information about the host machine, the host program, and the connection status.
*/
typedef struct sHostInfo
{
    int           bFoundHost;              //!< True = have talked to Seeker
    int           LatestConfirmationTime;  //!< Time of last receipt from Seeker
    char          szHostMachineName[128];  //!< Name of machine Seeker is running on
    unsigned char HostMachineAddress[4];   //!< IP Address of that machine
    char          szHostProgramName[128];  //!< Name of module communicating with
    unsigned char HostProgramVersion[4];   //!< Version number of that module

} sHostInfo;


//==================================================================

//! The rudimentary description of a skeleton's bones and hierarchy.
/*!
This description is defined by szSegmentNames[iSegment], and iParent[iSegment]
*/
typedef struct sHierarchy
{
    int            nSegments;         //!< Number of segments
    char** szSegmentNames;    //!< Array of segment names
    int* iParents;          //!< Array of segment parents (defines the hierarchy)

} sHierarchy;


//==================================================================

//! The description of a single tracking object that will have streaming data.
/*!
This description includes the object's name, the marker names, the skeleton hierarchy, and the DOF names.
*/
typedef struct sBodyDef
{
    char* szName;            //!< Name of the object

    int            nMarkers;          //!< Number of markers
    char** szMarkerNames;     //!< Array of marker names

    sHierarchy     Hierarchy;         //!< The Skeleton description for HTR data

    int            nDofs;             //!< Number of degrees of freedom
    char** szDofNames;        //!< Array of degrees of freedom names

} sBodyDef;


//==================================================================

//! The description of all the data that will stream from Seeker.
/*!
This description includes all the body descriptions, the analog channels,
and the number of forceplates.
*/
typedef struct sBodyDefs
{
    int            nBodyDefs;               //!< Number of bodies being tracked
    sBodyDef       BodyDefs[MAX_N_BODIES];  //!< The definition of each body

    int            nAnalogChannels;         //!< The number of active analog channels
    char** szAnalogChannelNames;    //!< The names given to each channel

    int            nForcePlates;            //!< The number of active forceplates

    void* AllocatedSpace;          //!< Private space (DON'T TOUCH)

} sBodyDefs;


//==================================================================

//! A structure containing ALL the data to drive one markerset.
/*!
This contains the markerset's name, the marker positions, the segment positions relative to each segment's parent, and the DOFs.
*/
typedef struct sBodyData
{
    char           szName[128];          //!< For dynamic matching of objects.

    int            nMarkers;             //!< Number of markers defined
    tMarkerData* Markers;              //!< [nMarkers][3] array.  Markers[iMarker][0] == XEMPTY means no data.
    float          fAvgMarkerResidual;   //!< Average residual of the marker triangulations

    int            nSegments;            //!< Number of segments
    tSegmentData* Segments;             //!< [nSegments][7] array

    int            nDofs;                //!< Number of degrees-of-freedom
    tDofData* Dofs;                 //!< Array of degree-of-freedom angles
    float          fAvgDofResidual;      //!< Average residual from the solve
    int            nIterations;          //!< Number of iterations to solve

    int            ZoomEncoderValue;     //!< Zoom value from the Camera Tracker Encoder
    int            FocusEncoderValue;    //!< Focus value from the Camera Tracker Encoder

} sBodyData;


//==================================================================

//! All the analog data for one frame's worth of time.
/*!
This includes the raw analog samples, processed forces, and also angle encoder values (if available).
*/
typedef struct sAnalogData
{
    int            nAnalogChannels;  //!< Total number of active channels
    int            nAnalogSamples;   //!< The number of samples in the current frame
    short* AnalogSamples;    //!< The data: nChannels * nSamples of these

    int            nForcePlates;     //!< Total number of active forceplates
    int            nForceSamples;    //!< The number of samples in the current frame
    tForceData* Forces;           //!< The forces: nForcePlates * nForceSamples of these

    int            nAngleEncoders;      //!< Number of encoders
    int            nAngleEncoderSamples;//!< Number of samples per encoder
    double* AngleEncoderSamples; //!< The angles: nEncoders*nEncoderSamples of these

} sAnalogData;


//==================================================================

//! The recording status tells us the frame numbers and capture filename.
typedef struct sRecordingStatus
{
    int            bRecording;   //!< 0=Not Recording, anything else=Recording
    int            iFirstFrame;  //!< The frame number of the first data frame to be recorded from Seeker Live Mode
    int            iLastFrame;   //!< The frame number of the last data frame to be recorded from Seeker Live Mode
    char           szFilename[256]; //!< The full capture filename

} sRecordingStatus;


typedef struct sTimeCode
{
    int            iStandard;   //!< 0=None, 1=SMPTE, 2=FILM, 3=EBU, 4=SystemClock
    int            iHours;
    int            iMinutes;
    int            iSeconds;
    int            iFrames;

} sTimeCode;

//==================================================================

//! A structure containing the current local machine time and you should Ensure the time synchronization
/*!
This struct has difference with the tm struct in time.h where it's month range is [1,12]
*/
typedef struct sAbsoluteTimeStamp
{
    bool isValid; // valid Flag
    int tm_msec; // millisecond - [0, 999]
    int tm_sec;   // seconds after the minute - [0, 60] including leap second
    int tm_min;   // minutes after the hour - [0, 59]
    int tm_hour;  // hours since midnight - [0, 23]
    int tm_mday;  // day of the month - [1, 31]
    int tm_mon;   // months since January - [1, 12]
    int tm_year;  // current year
    int tm_isdst; // Summer label.This value is positive if it is available in summer, zero if it is not, and negative if no information is available
} sAbsoluteTimeStamp;

//==================================================================

//! ALL the data for one frame streamed from Seeker.
/*!
This include the two items that describe the frame. The first is the frame number.
The second is the time delay measuring the delay between the real world action and the host sending this frame.
The actual data for the frame includes the data for each body, the unidentified markers, and data that is
associated with the analog captures.
*/
typedef struct sFrameOfData
{

    int            iFrame;                  //!< Seeker's frame number
    float          fDelay;                  //!< Total time (seconds) from Camera to the Host sending the data

    int            nBodies;                 //!< The bodies should match the descriptions
    sBodyData      BodyData[MAX_N_BODIES];  //!< The data for each body

    int            nUnidentifiedMarkers;    //!< Number of unrecognized markers
    tMarkerData* UnidentifiedMarkers;     //!< The unrecognized markers

    sAnalogData    AnalogData;              //!< The analog data packaged

    sRecordingStatus RecordingStatus;       //!< Info about name and frames being recorded

    sTimeCode      TimeCode;                //!< For system-wide frame alignment
    sAbsoluteTimeStamp TimeStamp;           //!< For Sync external device

} sFrameOfData;


//==================================================================

/** This function returns a 4-byte version number.
 *
 * \param Version - An array of four bytes: ModuleID, Major, Minor, Bugfix
 *
 * \return RC_Okay
*/
Seeker_DLL int Seeker_GetSdkVersion(unsigned char Version[4]);

//==================================================================

/** This function sets the filter level of the LogMessages.
 *
 *  The default verbosity level is VL_Warning.
 *
 * \param iLevel - one of the maVerbosityLevel enum values.
 *
 * \return RC_Okay
*/
Seeker_DLL int Seeker_SetVerbosityLevel(int iLevel);

//==================================================================

/**   The user supplied function handles text messages posted from within the SDK.
 *
 *    Logging messages is done as a utility to help code and/or run using the SDK.
 *    Various messages get posted for help with error conditions or events that happen.
 *    Each message has a Log-Level assigned to it so the user can.
 *  \sa Seeker_SetVerbosityLevel
 *
 *
 *  \param  MyFunction - This user defined function handles messages from the SDK.
 *
 *  \return maReturnCode - RC_Okay
*/
Seeker_DLL int Seeker_SetErrorMsgHandlerFunc(void (*MyFunction)(int iLogLevel, char* szLogMessage));

//==================================================================

/**   The user supplied function will be called whenever a frame of data arrives.
 *
 *    The ethernet servicing is done via a thread created
 *    when the connection to Seeker is made.  This function is
 *    called from that thread.  Some tasks are not sharable
 *    directly across threads.  Window redrawing, for example,
 *    should be done via events or messages.
 *
 *  \param MyFunction - This user supply callback function handles the streaming data
 *
 *  \return maReturnCode - RC_Okay
 *
 *    Notes: The data parameter points to "hot" data. That frame of data
 *           will be overwritten with the next call to the callback function.
*/
Seeker_DLL int Seeker_SetDataHandlerFunc(void (*MyFunction)(sFrameOfData* pFrameOfData));

//==================================================================

/**   The user supplied function will be called whenever a frame of data arrives.
*
*    The ethernet servicing is done via a thread created
*    when the connection to Seeker is made.  This function is
*    called from that thread.  Some tasks are not sharable
*    directly across threads.  Window redrawing, for example,
*    should be done via events or messages.
*
*  \param MyFunction - This user supply callback function handles the streaming data
*  \param pUserData -  Supply parameter Defined by User
*  \return maReturnCode - RC_Okay
*
*    Notes: The data parameter points to "hot" data. That frame of data
*           will be overwritten with the next call to the callback function.
*/
Seeker_DLL int Seeker_SetDataHandlerFunc_User(void (*MyFunction)(sFrameOfData* pFrameOfData, void* pUserData), void* pUserData = 0);

//==================================================================

/**   This function specifies the port on which Seeker listens for commands.
 *
 *    Newer versions of Seeker allow the user to choose on which port
 *    Seeker listens for commands. This function allows you to tell the
 *    SDK which port to send commands to. This function must be called
 *    before Seeker_Initialize() is called.
 *
 *  \param port - The port number to use for the multicast port
 *
 *  \return maReturnCode - RC_Okay
 *
 *    as part of his work on creating the Seeker_ros ROS node.
 */
Seeker_DLL int Seeker_SetSeekerPort(const unsigned short port);

//==================================================================

/**   This function specifies the multicast port on which Seeker will stream data.
 *
 *    Newer versions of Seeker allow the user to choose on which port
 *    Seeker will stream data. This function allows you to tell the
 *    SDK which port to listen to. This is especially useful on Linux
 *    machines, since the default port is 1001 and ports below 1024
 *    require root permissions to open. This function must be called
 *    before Seeker_Initialize() is called.
 *
 *  \param port - The port number to use for the multicast port
 *
 *  \return maReturnCode - RC_Okay
 *
 *    Notes: This function was added to the SDK by Daniel Koch on 8 Feb 2014
 *    as part of his work on creating the Seeker_ros ROS node.
 */
Seeker_DLL int Seeker_SetMultiCastPort(const unsigned short port);

//==================================================================

/**   This function defines the connection routes to talk to Seeker.
 *
 *    Machines can have more than one ethernet interface.  This function
 *    is used to either set the ethernet interface to use, or to let
 *    the SDK auto-select the local interface, and/or the Seeker host.
 *    This function should only be called once at startup.
 *
 *  \param szMyNicCardAddress - "a.b.c.d" or HostName.  "" and NULL mean AutoSelect
 *
 *  \param szSeekerNicCardAddress - "a.b.c.d" or HostName.  "" and NULL mean AutoSelect
 *
 *  \return maReturnCode - RC_Okay, RC_ApiError, RC_NetworkError, RC_GeneralError
*/
Seeker_DLL int Seeker_Initialize(const char* szMyNicCardAddress, const char* szSeekerNicCardAddress);  // Hostname or IP Address or NULL (auto find)

//==================================================================

/** This function gets information about the connection to Seeker
 *
 *  This function returns IP-Address information and Seeker version information.
 *  The version info can be used to handle incompatible changes in either our code
 *  or your code.
 *
 * \param pHostInfo - Structure containing connection information
 *
 * \return RC_Okay, RC_NetworkError
*/
Seeker_DLL int Seeker_GetHostInfo(sHostInfo* pHostInfo);

//==================================================================

/** This function stops all activity of the SDK.
 *
 *  This function should be called once before exiting.
*/
Seeker_DLL int Seeker_Exit();

//==================================================================

/**   This function sends commands to Seeker and returns a response.
 *
 *    This function is an extendable interface between the Client programs
 *    and the Host (Seeker) program.  The commands are sent as readable text strings.
 *    The response is returned unaltered.
 *
 * \param szCommand - The request to send the Seeker
 * \param ppResponse - The reply
 * \param pnBytes - The number of bytes in the response
 *
 \verbatim
Example:
    void *pResponse=NULL;
    Seeker_Request("GetSeekerFrameRate", &pResponse, sizeof(void*));
    fFrameRate = *(float*)pResponse;
\endverbatim
 *
 * \return RC_Okay, RC_TimeOut, RC_NotRecognized, RC_GeneralError
*/
Seeker_DLL int Seeker_Request(const char* szCommand, void** ppResponse, int* pnBytes);  // Friendly extendable command function.

//==================================================================

/**   This function queries Seeker for its set of tracking objects.
 *
 *  \return sBodyDefs* - This is a pointer to the internal storage of
 *                       the results of the latest call to this function.
 *
 *  \sa Seeker_FreeBodyDefs
*/
Seeker_DLL sBodyDefs* Seeker_GetBodyDefs();      // The description of what we are tracking.

//==================================================================

/** This function frees the memory allocated by Seeker_GetBodyDefs
 *
 *  The data within the structure is freed and also the structure itself.

 * \param pBodyDefs - The item to free.
 *
 * \return RC_Okay
*/
Seeker_DLL int Seeker_FreeBodyDefs(sBodyDefs* pBodyDefs);

//==================================================================

/** This function polls Seeker for the current frame
 *
 *  The SDK user has the streaming data available via the callback function.
 *  In addition, this function is available to get a frame directly.
 *
 *  Note: Seeker considers the current frame to be the latest LiveMode frame completed or,
 *        if not in LiveMode, the current frame is the one that is displayed on the screen.
 *
 * \return sFrameOfData
*/
Seeker_DLL sFrameOfData* Seeker_GetCurrentFrame();  // Can POLL for the current frame.

//==================================================================

/** This function copies a frame of data.
 *
 *  The Destination frame should start initialized to all zeros.  The CopyFrame
 *  and FreeFrame functions will handle the memory allocations necessary to fill
 *  out the data.
 *
 * \param pSrc - The frame to copy FROM.
 * \param pDst - The frame to copy TO
 *
 * \return RC_Okay, RC_MemoryError
*/
Seeker_DLL int Seeker_CopyFrame(const sFrameOfData* pSrc, sFrameOfData* pDst);  // Allocates or reallocates pointers

//==================================================================

/** This function frees memory within the structure.
 *
 *  The sFrameOfData structure includes pointers to various pieces of data.
 *  That data is dynamically allocated or reallocated to be consistent with
 *  the data that has arrived from Seeker.  To properly use the sFrameOfData
 *  structure, you should use the utility functions supplied.  It is possible
 *  to reuse sFrameOfData variables without ever freeing them.  The SDK will
 *  reallocate the components for you.
 *
 * \param pFrame - The frame of data to free.
 *
 * \return RC_Okay
*/
Seeker_DLL int Seeker_FreeFrame(sFrameOfData* pFrame);

//==================================================================

/** This function pushes a skeleton definition to Seeker.
 *
 *  A skeleton, defined in an animation package can be used to start
 *  a skeleton model definition in Seeker.  The hierarchy and starting
 *  pose can come from the animation package.  The rest of the details
 *  of the skeleton get filled out in the Seeker interface.  The parameters
 *  to this function match the parameters defining the HTR data that
 *  normally gets sent through the SDK2.
 *
 * \param pHierarchy - The number of segments, their names and parent child
                       relationships.
 * \param pFrame - One frame of HTR data dimensioned according to the number
 *                 of segments defined in the pHierarchy parameter.
 *
 * \return - RC_Okay, RC_NetworkError
*/
Seeker_DLL int Seeker_SendHtr(sHierarchy* pHierarchy, tSegmentData* pFrame);    // Push a skeleton definition to Seeker

Seeker_DLL int Seeker_SetMetered(bool bActive, float fFixedLatency);

//==================================================================
// Euler angle utility functions
//==================================================================

#define ZYX_ORDER 1
#define XYZ_ORDER 2
#define YXZ_ORDER 3
#define YZX_ORDER 4
#define ZXY_ORDER 5
#define XZY_ORDER 6

// Special rotation orders
#define XYX_ORDER 7
#define XZX_ORDER 8
#define YZY_ORDER 9
#define YXY_ORDER 10
#define ZXZ_ORDER 11
#define ZYZ_ORDER 12


//==================================================================

/** This function constructs a rotation matrix from three Euler angles.
 *
 *  This function and its inverse are utility functions for processing
 *  the HTR rotations we send in each frame of data. We send Euler angles
 *  in ZYX format (some interpretations would call it XYZ). Using these
 *  conversion utilities should protect against any misinterpretations.
 *
 * \param matrix - 3x3 rotation matrix.
 * \param iRotationOrder - one of:
 *
 *        ZYX_ORDER
 *        XYZ_ORDER
 *        YXZ_ORDER
 *        YZX_ORDER
 *        ZXY_ORDER
 *        XZY_ORDER
 *
 * \param angles - the angles in degrees.
 *
 */
Seeker_DLL void  Seeker_ConstructRotationMatrix(
        double angles[3],
        int iRotationOrder,
        double matrix[3][3]);

//==================================================================

/** This function decodes a rotation matrix into three Euler angles.
 *
 *  This function and its inverse are utility functions for processing
 *  the HTR rotations we send in each frame of data. We send Euler angles
 *  in ZYX format (some interpretations would call it XYZ). Using these
 *  conversion utilities should protect against any misinterpretations.
 *
 * \param matrix - 3x3 rotation matrix.
 * \param iRotationOrder - one of:
 *
 *        ZYX_ORDER
 *        XYZ_ORDER
 *        YXZ_ORDER
 *        YZX_ORDER
 *        ZXY_ORDER
 *        XZY_ORDER
 *
 * \param angles - the angles in degrees.
 *
*/
Seeker_DLL void  Seeker_ExtractEulerAngles(
        double matrix[3][3],
        int    iRotationOrder,
        double angles[3]);

#ifdef __cplusplus

}

#endif

#endif


