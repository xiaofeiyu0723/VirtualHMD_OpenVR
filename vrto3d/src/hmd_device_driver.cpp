/*
 * This file is part of VRto3D.
 *
 * VRto3D is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * VRto3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with VRto3D. If not, see <http://www.gnu.org/licenses/>.
 */
#define WIN32_LEAN_AND_MEAN

#include "hmd_device_driver.h"
#include "key_mappings.h"
#include "json_manager.h"
#include "app_id_mgr.h"
#include "driverlog.h"
#include "vrmath.h"

#include <string>
#include <sstream>
#include <ctime>

#include <winsock2.h>
#pragma comment (lib, "WSock32.Lib")
#include <windows.h>

inline vr::HmdQuaternion_t HmdQuaternion_Init(double w, double x, double y, double z)
{
    vr::HmdQuaternion_t quat;
    quat.w = w;
    quat.x = x;
    quat.y = y;
    quat.z = z;
    return quat;
}

inline vr::HmdQuaternion_t EulerAngleToQuaternion(double Yaw, double Pitch, double Roll)
{
    vr::HmdQuaternion_t q;
    // Abbreviations for the various angular functions
    double cy = cos(Yaw * 0.5);
    double sy = sin(Yaw * 0.5);
    double cp = cos(Pitch * 0.5);
    double sp = sin(Pitch * 0.5);
    double cr = cos(Roll * 0.5);
    double sr = sin(Roll * 0.5);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

//-----------------------------------------------------------------------------
// Purpose: Signify Operation Success
//-----------------------------------------------------------------------------
static void BeepSuccess()
{
    // High beep for success
    Beep(1800, 400);
}


// Load settings from default.vrsettings
static const char *stereo_main_settings_section = "driver_vrto3d";

MockControllerDeviceDriver::MockControllerDeviceDriver()
{
    // Keep track of whether Activate() has been called
    is_active_ = false;
    vr::DriverPose_t curr_pose_ = { 0 };
    app_name_ = "";

    auto* vrs = vr::VRSettings();
    JsonManager json_manager;
    json_manager.EnsureDefaultConfigExists();

    char model_number[ 1024 ];
    vrs->GetString( stereo_main_settings_section, "model_number", model_number, sizeof( model_number ) );
    stereo_model_number_ = model_number;
    char serial_number[ 1024 ];
    vrs->GetString( stereo_main_settings_section, "serial_number", serial_number, sizeof( serial_number ) );
    stereo_serial_number_ = serial_number;

    DriverLog( "VRto3D Model Number: %s", stereo_model_number_.c_str() );
    DriverLog( "VRto3D Serial Number: %s", stereo_serial_number_.c_str() );

    // Display settings
    StereoDisplayDriverConfiguration display_configuration{};
    display_configuration.window_x = 0;
    display_configuration.window_y = 0;
    json_manager.LoadParamsFromJson(display_configuration);

    // Profile settings
    json_manager.LoadProfileFromJson(DEF_CFG, display_configuration);

    // Instantiate our display component
    stereo_display_component_ = std::make_unique< StereoDisplayComponent >( display_configuration );

    DriverLog("Default Config Loaded\n");
}

//OpenTrack vars
double Yaw = 0, Pitch = 0, Roll = 0;
double pX = 0, pY = 0, pZ = 0;
struct TOpenTrack {
    double X;
    double Y;
    double Z;
    double Yaw;
    double Pitch;
    double Roll;
};
TOpenTrack OpenTrack;
//WinSock
SOCKET socketS;
int bytes_read;
struct sockaddr_in from;
int fromlen;
bool SocketActivated = false;

std::thread* pSocketThread = NULL;

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------

//double DegToRad(double f) {
//    return f * (3.14159265358979323846 / 180);
//}

void WinSockReadFunc()
{
    while (SocketActivated) {
        //Read UDP socket with OpenTrack data
        memset(&OpenTrack, 0, sizeof(OpenTrack));
        bytes_read = recvfrom(socketS, (char*)(&OpenTrack), sizeof(OpenTrack), 0, (sockaddr*)&from, &fromlen);

        if (bytes_read > 0) {
            Yaw = DEG_TO_RAD(OpenTrack.Yaw);
            Pitch = DEG_TO_RAD(OpenTrack.Pitch);
            Roll = DEG_TO_RAD(OpenTrack.Roll);
            pX = OpenTrack.X;
            pY = OpenTrack.Y;
            pZ = OpenTrack.Z;
        }
        else Sleep(1);
    }
}



//-----------------------------------------------------------------------------
// Purpose: Initialize all settings and notify SteamVR
//-----------------------------------------------------------------------------
vr::EVRInitError MockControllerDeviceDriver::Activate( uint32_t unObjectId )
{
    device_index_ = unObjectId;
    is_active_ = true;
    is_on_top_ = false;

    // A list of properties available is contained in vr::ETrackedDeviceProperty.
    auto* vrp = vr::VRProperties();
    auto* vrs = vr::VRSettings();
    vr::PropertyContainerHandle_t container = vrp->TrackedDeviceToPropertyContainer( device_index_ );
    vrp->SetStringProperty( container, vr::Prop_ModelNumber_String, stereo_model_number_.c_str() );
    vrp->SetStringProperty( container, vr::Prop_ManufacturerName_String, "VRto3D");
    vrp->SetStringProperty( container, vr::Prop_TrackingFirmwareVersion_String, "1.0");
    vrp->SetStringProperty( container, vr::Prop_HardwareRevision_String, "1.0");

    // Display settings
    vrp->SetFloatProperty( container, vr::Prop_UserIpdMeters_Float, stereo_display_component_->GetConfig().depth);
    vrp->SetFloatProperty( container, vr::Prop_UserHeadToEyeDepthMeters_Float, 0.f);
    vrp->SetFloatProperty( container, vr::Prop_DisplayFrequency_Float, stereo_display_component_->GetConfig().display_frequency * 1.5f );
    vrp->SetFloatProperty( container, vr::Prop_SecondsFromVsyncToPhotons_Float, stereo_display_component_->GetConfig().display_latency);
    vrp->SetFloatProperty( container, vr::Prop_SecondsFromPhotonsToVblank_Float, 0.0);
    vrp->SetBoolProperty( container, vr::Prop_ReportsTimeSinceVSync_Bool, false);
    vrp->SetBoolProperty( container, vr::Prop_IsOnDesktop_Bool, !stereo_display_component_->GetConfig().debug_enable);
    vrp->SetBoolProperty( container, vr::Prop_DisplayDebugMode_Bool, stereo_display_component_->GetConfig().debug_enable);
    vrp->SetBoolProperty( container, vr::Prop_HasDriverDirectModeComponent_Bool, false);
    if (stereo_display_component_->GetConfig().depth_gauge)
    {
        vrp->SetFloatProperty(container, vr::Prop_DashboardScale_Float, 1.0f);
    }
    else
    {
        vrp->SetFloatProperty(container, vr::Prop_DashboardScale_Float, 0.0f);
    }

    // Set the chaperone JSON property
    // Get the current time
    std::time_t t = std::time(nullptr);
    std::tm tm;
    localtime_s(&tm, &t);
    // Construct the JSON string with variables
    std::stringstream ss;
    ss << R"(
        {
           "jsonid" : "chaperone_info",
           "universes" : [
              {
                 "collision_bounds" : [
                    [
                       [ -1.0, 0.0, -1.0 ],
                       [ -1.0, 3.0, -1.0 ],
                       [ -1.0, 3.0, 1.0 ],
                       [ -1.0, 0.0, 1.0 ]
                    ],
                    [
                       [ -1.0, 0.0, 1.0 ],
                       [ -1.0, 3.0, 1.0 ],
                       [ 1.0, 3.0, 1.0 ],
                       [ 1.0, 0.0, 1.0 ]
                    ],
                    [
                       [ 1.0, 0.0, 1.0 ],
                       [ 1.0, 3.0, 1.0 ],
                       [ 1.0, 3.0, -1.0 ],
                       [ 1.0, 0.0, -1.0 ]
                    ],
                    [
                       [ 1.0, 0.0, -1.0 ],
                       [ 1.0, 3.0, -1.0 ],
                       [ -1.0, 3.0, -1.0 ],
                       [ -1.0, 0.0, -1.0 ]
                    ]
                 ],
                 "play_area" : [ 2.0, 2.0 ],
                 "seated" : {
                    "translation" : [ 0.0, 0.5, 0.0 ],
                    "yaw" : 0.0
                 },
                 "standing" : {
                    "translation" : [ 0.0, 1.0, 0.0 ],
                    "yaw" : 0.0
                 },
                 "time" : ")" << std::put_time(&tm, "%a %b %d %H:%M:%S %Y") << R"(",
                 "universeID" : "64"
              }
           ],
           "version" : 5
        }
        )";
    // Convert the stringstream to a string
    std::string chaperoneJson = ss.str();
    // Set the chaperone JSON property
    vrp->SetStringProperty(container, vr::Prop_DriverProvidedChaperoneJson_String, chaperoneJson.c_str());
    vrp->SetUint64Property(container, vr::Prop_CurrentUniverseId_Uint64, 64);
    vrs->SetInt32(vr::k_pch_CollisionBounds_Section, vr::k_pch_CollisionBounds_Style_Int32, vr::COLLISION_BOUNDS_STYLE_NONE);
    vrs->SetBool(vr::k_pch_CollisionBounds_Section, vr::k_pch_CollisionBounds_GroundPerimeterOn_Bool, false);

    // Miscellaneous settings
    vrp->SetBoolProperty( container, vr::Prop_WillDriftInYaw_Bool, false);
    vrp->SetBoolProperty( container, vr::Prop_DeviceIsWireless_Bool, false);
    vrp->SetBoolProperty( container, vr::Prop_DeviceIsCharging_Bool, false);
    vrp->SetBoolProperty( container, vr::Prop_ContainsProximitySensor_Bool, false);
    vrp->SetBoolProperty( container, vr::Prop_DeviceCanPowerOff_Bool, false);

    // set proximity senser to always on, always head present
    vr::VRInputComponentHandle_t  prox;
    vr::VRDriverInput()->CreateBooleanComponent(container, "/proximity", &prox);
    vr::VRDriverInput()->UpdateBooleanComponent(prox, true, 0.0);
    
    // Miscellaneous settings
    vrs->SetBool(vr::k_pch_DirectMode_Section, vr::k_pch_DirectMode_Enable_Bool, false);
    vrs->SetFloat(vr::k_pch_Power_Section, vr::k_pch_Power_TurnOffScreensTimeout_Float, 86400.0f);
    vrs->SetBool(vr::k_pch_Power_Section, vr::k_pch_Power_PauseCompositorOnStandby_Bool, false);
    vrs->SetBool(vr::k_pch_Dashboard_Section, vr::k_pch_Dashboard_EnableDashboard_Bool, false);
    vrs->SetBool(vr::k_pch_Dashboard_Section, vr::k_pch_Dashboard_ArcadeMode_Bool, true);
    vrs->SetBool(vr::k_pch_Dashboard_Section, "allowAppQuitting", false);
    vrs->SetBool(vr::k_pch_Dashboard_Section, "autoShowGameTheater", false);
    vrs->SetBool(vr::k_pch_Dashboard_Section, "showDesktop", false);
    vrs->SetBool(vr::k_pch_Dashboard_Section, "showPowerOptions", false);
    vrs->SetBool(vr::k_pch_Dashboard_Section, "inputCaptureEnabled", false);
    vrs->SetBool(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_EnableHomeApp, false);
    vrs->SetBool(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_MirrorViewVisibility_Bool, false);
    vrs->SetBool(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_EnableSafeMode, false);
    vrs->SetBool(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_DisplayDebug_Bool, false);
    vrs->SetBool(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_MotionSmoothing_Bool, false);
    vrs->SetBool(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_DisableAsyncReprojection_Bool, true);
    vrs->SetBool(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_AllowSupersampleFiltering_Bool, false);
    vrs->SetBool(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_SupersampleManualOverride_Bool, true);
    vrs->SetBool(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_ForceFadeOnBadTracking_Bool, false);
    
    //Open UDP port for receive data from OpenTrack ("UDP over network", 127.0.0.1, 4242)
    WSADATA wsaData;
    int iResult;
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult == 0) {
        struct sockaddr_in local;
        fromlen = sizeof(from);
        local.sin_family = AF_INET;
        local.sin_port = htons(4242);
        local.sin_addr.s_addr = INADDR_ANY;

        socketS = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

        u_long nonblocking_enabled = true;
        ioctlsocket(socketS, FIONBIO, &nonblocking_enabled);

        if (socketS != INVALID_SOCKET) {

            iResult = bind(socketS, (sockaddr*)&local, sizeof(local));

            if (iResult != SOCKET_ERROR) {
                SocketActivated = true;
                pSocketThread = new std::thread(WinSockReadFunc);
            }
            else {
                WSACleanup();
                SocketActivated = false;
            }

        }
        else {
            WSACleanup();
            SocketActivated = false;
        }

    }
    else
    {
        WSACleanup();
        SocketActivated = false;
    }

    pose_thread_ = std::thread(&MockControllerDeviceDriver::PoseUpdateThread, this);

    HANDLE thread_handle = pose_thread_.native_handle();

    // Set the thread priority
    if (!SetThreadPriority(thread_handle, THREAD_PRIORITY_HIGHEST)) {
        // Handle error if setting priority fails
        DriverLog("Failed to set thread priority: %d\n", GetLastError());
    }

    DriverLog("Activation Complete\n");

    return vr::VRInitError_None;
}

//-----------------------------------------------------------------------------
// Purpose: Return StereoDisplayComponent as vr::IVRDisplayComponent
//-----------------------------------------------------------------------------
void *MockControllerDeviceDriver::GetComponent( const char *pchComponentNameAndVersion )
{
    if ( strcmp( pchComponentNameAndVersion, vr::IVRDisplayComponent_Version ) == 0 )
    {
        return stereo_display_component_.get();
    }

    return nullptr;
}

//-----------------------------------------------------------------------------
// Purpose: This is called by vrserver when a debug request has been made from an application to the driver.
// What is in the response and request is up to the application and driver to figure out themselves.
//-----------------------------------------------------------------------------
void MockControllerDeviceDriver::DebugRequest( const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize )
{
    if ( unResponseBufferSize >= 1 )
        pchResponseBuffer[ 0 ] = 0;
}

//-----------------------------------------------------------------------------
// Purpose: Static Pose with pitch & yaw adjustment
//-----------------------------------------------------------------------------
void MockControllerDeviceDriver::PoseUpdateThread()
{
    auto lastTime = std::chrono::high_resolution_clock::now();
    vr::DriverPose_t pose = { 0 };
    while (is_active_)
    {
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto deltaTime = std::chrono::duration_cast<std::chrono::duration<float>>(currentTime - lastTime).count();
        lastTime = currentTime;

        if (SocketActivated) {
            pose.poseIsValid = true;
            pose.result = vr::ETrackingResult::TrackingResult_Running_OK;
            pose.deviceIsConnected = true;
        }
        else
        {
            pose.poseIsValid = false;
            pose.result = vr::ETrackingResult::TrackingResult_Uninitialized;
            pose.deviceIsConnected = false;
        }

        pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
        pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);

        //Set head tracking rotation
        pose.qRotation = EulerAngleToQuaternion(Roll, -Yaw, Pitch);

        //Set position tracking
        pose.vecPosition[0] = pX * 0.01;
        pose.vecPosition[1] = pZ * 0.01;
        pose.vecPosition[2] = pY * 0.01;
        pose.poseIsValid = true;
        pose.deviceIsConnected = true;
        pose.result = vr::TrackingResult_Running_OK;
        pose.shouldApplyHeadModel = false;
        pose.willDriftInYaw = false;
        pose.poseTimeOffset = 0;

        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(device_index_, pose, sizeof(vr::DriverPose_t));

        //  125Hz
        auto endTime = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - currentTime);
        std::this_thread::sleep_for(std::chrono::milliseconds(8) - elapsed);
    }
}


//-----------------------------------------------------------------------------
// Purpose: Return current pose
//-----------------------------------------------------------------------------
vr::DriverPose_t MockControllerDeviceDriver::GetPose()
{
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return curr_pose_;
}

//-----------------------------------------------------------------------------
// Purpose: Load Game Specific Settings from Documents\My games\vrto3d\app_name_config.json
//-----------------------------------------------------------------------------
void MockControllerDeviceDriver::LoadSettings(const std::string& app_name)
{
    if (app_name != app_name_)
    {
        app_name_ = app_name;
        auto config = stereo_display_component_->GetConfig();
        // Attempt to get Steam App ID
        AppIdMgr app_id_mgr;
        auto app_id = app_id_mgr.GetRunningSteamGameAppID();
        if (!app_id.empty())
        {
            std::string vr_str = "steam.app." + app_id;
            vr::VRSettings()->SetBool(vr_str.c_str(), vr::k_pch_SteamVR_DisableAsyncReprojection_Bool, true);
        }
        // Attempt to read the JSON settings file
        JsonManager json_manager;
        if (json_manager.LoadProfileFromJson(app_name + "_config.json", config))
        {
            stereo_display_component_->LoadSettings(config, device_index_);
            DriverLog("Loaded %s profile\n", app_name.c_str());
            BeepSuccess();
        }
    }
}


//-----------------------------------------------------------------------------
// Purpose: Stub for Standby mode
//-----------------------------------------------------------------------------
void MockControllerDeviceDriver::EnterStandby()
{
    DriverLog( "HMD has been put into standby." );
}

//-----------------------------------------------------------------------------
// Purpose: Shutdown process
//-----------------------------------------------------------------------------
void MockControllerDeviceDriver::Deactivate()
{
    if ( is_active_.exchange( false ) )
    {
        pose_thread_.join();
        hotkey_thread_.join();
        focus_thread_.join();
    }

    // unassign our controller index (we don't want to be calling vrserver anymore after Deactivate() has been called
    device_index_ = vr::k_unTrackedDeviceIndexInvalid;
}


//-----------------------------------------------------------------------------
// DISPLAY DRIVER METHOD DEFINITIONS
//-----------------------------------------------------------------------------

StereoDisplayComponent::StereoDisplayComponent( const StereoDisplayDriverConfiguration &config )
    : config_( config ), depth_(config.depth), convergence_(config.convergence)
{
}

//-----------------------------------------------------------------------------
// Purpose: To inform vrcompositor if this display is considered an on-desktop display.
//-----------------------------------------------------------------------------
bool StereoDisplayComponent::IsDisplayOnDesktop()
{
    std::shared_lock<std::shared_mutex> lock(cfg_mutex_);
    return !config_.debug_enable;
}

//-----------------------------------------------------------------------------
// Purpose: To as vrcompositor to search for this display.
//-----------------------------------------------------------------------------
bool StereoDisplayComponent::IsDisplayRealDisplay()
{
    return false;
}

//-----------------------------------------------------------------------------
// Purpose: To inform the rest of the vr system what the recommended target size should be
//-----------------------------------------------------------------------------
void StereoDisplayComponent::GetRecommendedRenderTargetSize( uint32_t *pnWidth, uint32_t *pnHeight )
{
    std::shared_lock<std::shared_mutex> lock(cfg_mutex_);
    *pnWidth = config_.render_width;
    *pnHeight = config_.render_height;
}

//-----------------------------------------------------------------------------
// Purpose: Render in SbS or TaB Stereo3D
//-----------------------------------------------------------------------------
void StereoDisplayComponent::GetEyeOutputViewport( vr::EVREye eEye, uint32_t *pnX, uint32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight )
{
    std::shared_lock<std::shared_mutex> lock(cfg_mutex_);
    if (config_.reverse_enable)
    {
        eEye = static_cast<vr::EVREye>(!static_cast<bool> (eEye));
    }
    // Use Top and Bottom Rendering
    if (config_.tab_enable)
    {
        *pnX = 0;
        // Each eye will have full width
        *pnWidth = config_.window_width;
        // Each eye will have half height
        *pnHeight = config_.window_height / 2;
        if (eEye == vr::Eye_Left)
        {
            // Left eye viewport on the top half of the window
            *pnY = 0;
        }
        else
        {
            // Right eye viewport on the bottom half of the window
            *pnY = config_.window_height / 2;
        }
    }

    // Use Side by Side Rendering
    else
    {
        *pnY = 0;
        // Each eye will have half width
        *pnWidth = config_.window_width / 2;
        // Each eye will have full height
        *pnHeight = config_.window_height;
        if (eEye == vr::Eye_Left)
        {
            // Left eye viewport on the left half of the window
            *pnX = 0;
        }
        else
        {
            // Right eye viewport on the right half of the window
            *pnX = config_.window_width / 2;
        }
    }
}

//-----------------------------------------------------------------------------
// Purpose: Utilize the desired FoV, Aspect Ratio, and Convergence settings
//-----------------------------------------------------------------------------
void StereoDisplayComponent::GetProjectionRaw( vr::EVREye eEye, float *pfLeft, float *pfRight, float *pfTop, float *pfBottom )
{
    std::shared_lock<std::shared_mutex> lock(cfg_mutex_);
    // Convert horizontal FOV from degrees to radians
    float horFovRadians = tan((config_.fov * (M_PI / 180.0f)) / 2);

    // Calculate the vertical FOV in radians
    float verFovRadians = tan(atan(horFovRadians / config_.aspect_ratio));

    // Get convergence value
    float convergence = GetConvergence();

    // Calculate the raw projection values
    *pfTop = -verFovRadians;
    *pfBottom = verFovRadians;

    // Adjust the frustum based on the eye
    if (eEye == vr::Eye_Left) {
        *pfLeft = -horFovRadians + convergence;
        *pfRight = horFovRadians + convergence;
    }
    else {
        *pfLeft = -horFovRadians - convergence;
        *pfRight = horFovRadians - convergence;
    }
}

//-----------------------------------------------------------------------------
// Purpose: Don't distort any coordinates for Stereo3D
//-----------------------------------------------------------------------------
vr::DistortionCoordinates_t StereoDisplayComponent::ComputeDistortion( vr::EVREye eEye, float fU, float fV )
{
    vr::DistortionCoordinates_t coordinates{};
    coordinates.rfBlue[ 0 ] = fU;
    coordinates.rfBlue[ 1 ] = fV;
    coordinates.rfGreen[ 0 ] = fU;
    coordinates.rfGreen[ 1 ] = fV;
    coordinates.rfRed[ 0 ] = fU;
    coordinates.rfRed[ 1 ] = fV;
    return coordinates;
}
bool StereoDisplayComponent::ComputeInverseDistortion(vr::HmdVector2_t* pResult, vr::EVREye eEye, uint32_t unChannel, float fU, float fV)
{
    return false;
}

//-----------------------------------------------------------------------------
// Purpose: To inform vrcompositor what the window bounds for this virtual HMD are.
//-----------------------------------------------------------------------------
void StereoDisplayComponent::GetWindowBounds( int32_t *pnX, int32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight )
{
    std::shared_lock<std::shared_mutex> lock(cfg_mutex_);
    *pnX = config_.window_x;
    *pnY = config_.window_y;
    *pnWidth = config_.window_width;
    *pnHeight = config_.window_height;
}

//-----------------------------------------------------------------------------
// Purpose: To provide access to settings
//-----------------------------------------------------------------------------
StereoDisplayDriverConfiguration StereoDisplayComponent::GetConfig()
{
    std::shared_lock<std::shared_mutex> lock(cfg_mutex_);
    return config_;
}


//-----------------------------------------------------------------------------
// Purpose: To update the Depth value
//-----------------------------------------------------------------------------
void StereoDisplayComponent::AdjustDepth(float new_depth, bool is_delta, uint32_t device_index)
{
    float cur_depth = GetDepth();
    if (is_delta)
        new_depth += cur_depth;
    while (!depth_.compare_exchange_weak(cur_depth, new_depth, std::memory_order_relaxed));
    vr::PropertyContainerHandle_t container = vr::VRProperties()->TrackedDeviceToPropertyContainer(device_index);
    vr::VRProperties()->SetFloatProperty(container, vr::Prop_UserIpdMeters_Float, new_depth);
}


//-----------------------------------------------------------------------------
// Purpose: To update the Convergence value
//-----------------------------------------------------------------------------
void StereoDisplayComponent::AdjustConvergence(float new_conv, bool is_delta, uint32_t device_index)
{
    float cur_conv = GetConvergence();
    if (is_delta)
        new_conv += cur_conv;
    if (cur_conv == new_conv)
        return;
    while (!convergence_.compare_exchange_weak(cur_conv, new_conv, std::memory_order_relaxed));
    // Regenerate the Projection
    vr::HmdRect2_t eyeLeft, eyeRight;
    GetProjectionRaw(vr::Eye_Left, &eyeLeft.vTopLeft.v[0], &eyeLeft.vBottomRight.v[0], &eyeLeft.vTopLeft.v[1], &eyeLeft.vBottomRight.v[1]);
    GetProjectionRaw(vr::Eye_Right, &eyeRight.vTopLeft.v[0], &eyeRight.vBottomRight.v[0], &eyeRight.vTopLeft.v[1], &eyeRight.vBottomRight.v[1]);
    vr::VREvent_Data_t temp;
    vr::VRServerDriverHost()->SetDisplayProjectionRaw(device_index, eyeLeft, eyeRight);
    vr::VRServerDriverHost()->VendorSpecificEvent(device_index, vr::VREvent_LensDistortionChanged, temp, 0.0f);
}


//-----------------------------------------------------------------------------
// Purpose: Get Depth value
//-----------------------------------------------------------------------------
float StereoDisplayComponent::GetDepth()
{
    return depth_.load(std::memory_order_relaxed);
}


//-----------------------------------------------------------------------------
// Purpose: Get Convergence value
//-----------------------------------------------------------------------------
float StereoDisplayComponent::GetConvergence()
{
    return convergence_.load(std::memory_order_relaxed);
}

//-----------------------------------------------------------------------------
// Purpose: Toggle HMD Height for games that have incorrect HMD position
//-----------------------------------------------------------------------------
void StereoDisplayComponent::SetHeight()
{
    static float user_height = config_.hmd_height;

    std::unique_lock<std::shared_mutex> lock(cfg_mutex_);
    if (config_.hmd_height == user_height)
        config_.hmd_height = 0.1f;
    else
        config_.hmd_height = user_height;
}


//-----------------------------------------------------------------------------
// Purpose: Toggle Reset off
//-----------------------------------------------------------------------------
void StereoDisplayComponent::SetReset()
{
    std::unique_lock<std::shared_mutex> lock(cfg_mutex_);
}


//-----------------------------------------------------------------------------
// Purpose: Load Game Specific Settings from Documents\My games\vrto3d\app_name_config.json
//-----------------------------------------------------------------------------
void StereoDisplayComponent::LoadSettings(StereoDisplayDriverConfiguration& config, uint32_t device_index)
{
    // Apply loaded settings
    AdjustDepth(config.depth, false, device_index);
    AdjustConvergence(config.convergence, false, device_index);
    
    std::unique_lock<std::shared_mutex> lock(cfg_mutex_);
    config_ = config;
}
