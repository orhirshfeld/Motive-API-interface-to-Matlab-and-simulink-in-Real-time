
//==========================================================================----
//== Trackable Settings
//== Copyright NaturalPoint ==--
//==========================================================================----

#ifndef __TRACKABLESETTINGS_H__
#define __TRACKABLESETTINGS_H__

//==================================================================================-----

namespace Core
{
    const int kTrackableNameMaxLen = 32;
    const int kTrackableModelNameMaxLen = 256;
    class cIReader;
    class cIWriter;
}

class cTrackableSettings
{
public:
    cTrackableSettings();

    void  Save(Core::cIWriter *Serial) const;
    void  Load(Core::cIReader *Serial);

    wchar_t  mName     [Core::kTrackableNameMaxLen];
    wchar_t  mModelName[Core::kTrackableModelNameMaxLen];

    int   UserData;
    float ColorR;
    float ColorG;
    float ColorB;

    float MaxFrameRotation;               //== deprecated
    float MaxFrameTranslation;            //== deprecated

    bool  DynamicRotationConstraint;      //== deprecated
    bool  DynamicTranslationConstraint;   //== deprecated
    int   DynamicConstraintFrames;
    bool  StaticYawRotationConstraint;    //== deprecated
    bool  StaticPitchRotationConstraint;  //== deprecated
    bool  StaticRollRotationConstraint;   //== deprecated

    float YawGreaterThan;                 //== deprecated
    float YawLessThan;                    //== deprecated
    float PitchGreaterThan;               //== deprecated
    float PitchLessThan;                  //== deprecated
    float RollGreaterThan;                //== deprecated
    float RollLessThan;                   //== deprecated

    double Smoothing;                     

    bool   Visible;
    bool   DisplayUntracked;
    bool   DisplayPivot;
    bool   DisplayUntrackedMarkers;
    bool   DisplayMarkerQuality;
    bool   DisplayQuality;
    bool   DisplayTracked;
    bool   DisplayLabel;
    bool   DisplayOrientation;
    bool   DisplayModelReplace;

    float  ModelYaw;
    float  ModelPitch;
    float  ModelRoll;
    float  ModelX;
    float  ModelY;
    float  ModelZ;
    float  ModelScale;

    bool   DisplayPositionHistory;
    bool   DisplayOrientationHistory;
    int    DisplayHistoryLength;
    int    DisplayOrientationSpread;
    int    DisplayOrientationSize;

    //== Static constraint ==--

    bool    StaticOrientationConstraint;
    double  StaticConstraintX;
    double  StaticConstraintY;
    double  StaticConstraintZ;
    double  StaticConstraintAngle;

    // Solver settings
    bool    Enabled;
    float   MaxMarkerDeflection;
    int     MinimumMarkerCount;
    int     MinimumHitCount;         //== depricated
    float   Flexibility;
    bool    ShareMarkers;
    bool    Unique;
    int     DynamicOverride;         //== depricated
    int     StaticOverride;          //== depricated
    bool    PreciseOrientation;      //== depricated
    bool    ForceExhaustive;         //== depricated
    double  MaxCalculationTime;
    int     AcquisitionFrames;
};


#endif
