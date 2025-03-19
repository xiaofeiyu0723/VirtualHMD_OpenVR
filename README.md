# VirtualHMD_OpenVR

- This project is developed based on the [VRto3D](https://github.com/oneup03/VRto3D) and [OpenVR-OpenTrack](https://github.com/r57zone/OpenVR-OpenTrack), with functionality fully similar to the UDP version of OpenVR-OpenTrack. It fixes the red screen problem and mirror shifting problem in VRChat, and you can use the same way to send the position and rotation data of HMD to this driver.

## Configuration

- VRto3D has to be installed and SteamVR launched once for this config file to show up
- Modify the `Documents\My Games\vrto3d\
- ` for your setup
- Most changes made to this configuration require a restart of SteamVR to take effect

| Field Name          | Type    | Description                                                                                 | Default Value  |
|---------------------|---------|---------------------------------------------------------------------------------------------|----------------|
| `window_width`      | `int`   | The width of the application window.                                                        | `1920`         |
| `window_height`     | `int`   | The height of the application window.                                                       | `1080`         |
| `render_width`      | `int`   | The width to render per eye (can be higher or lower than the application window)            | `1920`         |
| `render_height`     | `int`   | The height to render per eye (can be higher or lower than the application window)           | `1080`         |
| `hmd_height` +      | `float` | The height of the simulated HMD.                                                            | `1.0`          |
| `aspect_ratio`      | `float` | The aspect ratio used to calculate vertical FoV                                             | `1.77778`      |
| `fov`               | `float` | The field of view (FoV) for the VR rendering.                                               | `120.0`         |
| `depth` +           | `float` | The max depth. Overrides VR's IPD field.                                                    | `0.5`          |
| `convergence` +     | `float` | Where the left and right images converge. Adjusts frustum.                                  | `0.02`         |
| `debug_enable`      | `bool`  | Borderless Windowed. Not 3DVision compatible. Breaks running some mods in OpenVR mode.      | `true`         |
| `display_latency`   | `float` | The display latency in seconds.                                                             | `0.011`        |
| `display_frequency` | `float` | The display refresh rate, in Hz.                                                            | `60.0`         |
| `user_depth` +      | `float` | The depth value for user setting # (replace # with integer number)                          | `0.0`          |
| `user_convergence` +| `float` | The convergence value for user setting # (replace # with integer number)                    | `0.0`         |

## Base Installation
- Install SteamVR
- Download the [latest release](xxx) and copy the `VirtualHMD` folder to your `Steam\steamapps\common\SteamVR\drivers` folder
- Launch SteamvR once to generate the `default_config.json` and you should see a 1080p SbS `Headset Window`
- Close SteamVR
- Edit the `Documents\My Games\vrto3d\default_config.json` as needed - [see what each setting does](#configuration)
    - Set your window resolution to match your fullscreen resolution
    - Set your render resolution per eye to what you want - can save some performance by reducing this. If your display is half-SbS or half-TaB, then you can try setting this to that half-resolution
    - Single Display Mode: make sure the `debug_enable` flag is set to `true` to make more games work (not 3DVision compatible)
- Try launching a VR game

## Notes
- SteamVR may still complain about Direct Display mode, but this can be safely dismissed
- Exiting SteamVR will "restart" Steam - this is normal
- Overlays generally won't work on this virtual HMD


#### Troubleshooting
- If you have used other SteamVR drivers that also create a virtual HMD, you will need to disable and/or uninstall them
    - Run SteamVR
    - On the SteamVR Status window, go to `Menu -> Settings`
    - Change to the `Startup / Shutdown` tab
    - Click `Manage Add-Ons`
    - Turn `Off` any virtual HMD drivers (ALVR, VRidge, OpenTrack, VCR, iVRy, etc)
    - if issues still arise, try a [Clean SteamVR Install](https://steamcommunity.com/app/250820/discussions/2/1640917625015598552/) and delete your `Steam\steamapps\common\SteamVR` folder
- If you have a VR headset and run into issues with this driver, here's some things to try:
    - Disconnect VR headset from computer
    - [Clean SteamVR Install](https://steamcommunity.com/app/250820/discussions/2/1640917625015598552/)
    - [Set SteamVR as OpenXR Runtime](https://www.vive.com/us/support/vs/category_howto/trouble-with-openxr-titles.html)

## Building
- Clone the code and initialize submodules
- Define `STEAM_PATH` environment variable with the path to your main Steam folder
- Open Solution in Visual Studio 2022
- Use the solution to build this driver
- Build output is automatically copied to your `SteamVR\drivers` folder
