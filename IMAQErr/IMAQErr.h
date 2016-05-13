#ifndef ___imaqerr_h___
#define ___imaqerr_h___

#include <string.h>
#include "NIIMAQdx.h"

void IMAQErrorMessage (int aError, char * aMessage)
{
    if( aError == (signed int)IMAQdxErrorSuccess) {  strcpy(aMessage,                        "Success"); return; }
    if( aError == (signed int)IMAQdxErrorSystemMemoryFull) {  strcpy(aMessage,               "Not enough memory."); return; }
    if( aError == (signed int)IMAQdxErrorInternal) {  strcpy(aMessage,                       "Internal error."); return; }
    if( aError == (signed int)IMAQdxErrorInvalidParameter) {  strcpy(aMessage,               "Invalid parameter."); return; }
    if( aError == (signed int)IMAQdxErrorInvalidPointer) {  strcpy(aMessage,                 "Invalid pointer."); return; }
    if( aError == (signed int)IMAQdxErrorInvalidInterface) {  strcpy(aMessage,               "Invalid camera session."); return; }
    if( aError == (signed int)IMAQdxErrorInvalidRegistryKey) {  strcpy(aMessage,             "Invalid registry key."); return; }
    if( aError == (signed int)IMAQdxErrorInvalidAddress) {  strcpy(aMessage,                 "Invalid address."); return; }
    if( aError == (signed int)IMAQdxErrorInvalidDeviceType) {  strcpy(aMessage,              "Invalid device type."); return; }
    if( aError == (signed int)IMAQdxErrorNotImplemented) {  strcpy(aMessage,                 "Not implemented."); return; }
    if( aError == (signed int)IMAQdxErrorCameraNotFound) {  strcpy(aMessage,                 "Camera not found."); return; }
    if( aError == (signed int)IMAQdxErrorCameraInUse) {  strcpy(aMessage,                    "Camera is already in use."); return; }
    if( aError == (signed int)IMAQdxErrorCameraNotInitialized) {  strcpy(aMessage,           "Camera is not initialized."); return; }
    if( aError == (signed int)IMAQdxErrorCameraRemoved) {  strcpy(aMessage,                  "Camera has been removed."); return; }
    if( aError == (signed int)IMAQdxErrorCameraRunning) {  strcpy(aMessage,                  "Acquisition in progress."); return; }
    if( aError == (signed int)IMAQdxErrorCameraNotRunning) {  strcpy(aMessage,               "No acquisition in progress."); return; }
    if( aError == (signed int)IMAQdxErrorAttributeNotSupported) {  strcpy(aMessage,          "Attribute not supported by the camera."); return; }
    if( aError == (signed int)IMAQdxErrorAttributeNotSettable) {  strcpy(aMessage,           "Unable to set attribute."); return; }
    if( aError == (signed int)IMAQdxErrorAttributeNotReadable) {  strcpy(aMessage,           "Unable to get attribute."); return; }
    if( aError == (signed int)IMAQdxErrorAttributeOutOfRange) {  strcpy(aMessage,            "Attribute value is out of range."); return; }
    if( aError == (signed int)IMAQdxErrorBufferNotAvailable) {  strcpy(aMessage,             "Requested buffer is unavailable."); return; }
    if( aError == (signed int)IMAQdxErrorBufferListEmpty) {  strcpy(aMessage,                "Buffer list is empty. Add one or more buffers."); return; }
    if( aError == (signed int)IMAQdxErrorBufferListLocked) {  strcpy(aMessage,               "Buffer list is already locked. Reconfigure acquisition and try again."); return; }
    if( aError == (signed int)IMAQdxErrorBufferListNotLocked) {  strcpy(aMessage,            "No buffer list. Reconfigure acquisition and try again."); return; }
    if( aError == (signed int)IMAQdxErrorResourcesAllocated) {  strcpy(aMessage,             "Transfer engine resources already allocated. Reconfigure acquisition and try again."); return; }
    if( aError == (signed int)IMAQdxErrorResourcesUnavailable) {  strcpy(aMessage,           "Insufficient transfer engine resources."); return; }
    if( aError == (signed int)IMAQdxErrorAsyncWrite) {  strcpy(aMessage,                     "Unable to perform asynchronous register write."); return; }
    if( aError == (signed int)IMAQdxErrorAsyncRead) {  strcpy(aMessage,                      "Unable to perform asynchronous register read."); return; }
    if( aError == (signed int)IMAQdxErrorTimeout) {  strcpy(aMessage,                        "Timeout."); return; }
    if( aError == (signed int)IMAQdxErrorBusReset) {  strcpy(aMessage,                       "Bus reset occurred during a transaction."); return; }
    if( aError == (signed int)IMAQdxErrorInvalidXML) {  strcpy(aMessage,                     "Unable to load camera's XML file."); return; }
    if( aError == (signed int)IMAQdxErrorFileAccess) {  strcpy(aMessage,                     "Unable to read/write to file."); return; }
    if( aError == (signed int)IMAQdxErrorInvalidCameraURLString) {  strcpy(aMessage,         "Camera has malformed URL string."); return; }
    if( aError == (signed int)IMAQdxErrorInvalidCameraFile) {  strcpy(aMessage,              "Invalid camera file."); return; }
    if( aError == (signed int)IMAQdxErrorGenICamError) {  strcpy(aMessage,                   "Unknown Genicam error."); return; }
    if( aError == (signed int)IMAQdxErrorFormat7Parameters) {  strcpy(aMessage,              "For format 7: The combination of speed) {  strcpy(aMessage,  image position) {  strcpy(aMessage,  image size) {  strcpy(aMessage,  and color coding is incorrect."); return; }
    if( aError == (signed int)IMAQdxErrorInvalidAttributeType) {  strcpy(aMessage,           "The attribute type is not compatible with the passed variable type."); return; }
    if( aError == (signed int)IMAQdxErrorDLLNotFound) {  strcpy(aMessage,                    "The DLL could not be found."); return; }
    if( aError == (signed int)IMAQdxErrorFunctionNotFound) {  strcpy(aMessage,               "The function could not be found."); return; }
    if( aError == (signed int)IMAQdxErrorLicenseNotActivated) {  strcpy(aMessage,            "License not activated."); return; }
    if( aError == (signed int)IMAQdxErrorCameraNotConfiguredForListener) {  strcpy(aMessage, "The camera is not configured properly to support a listener."); return; }
    if( aError == (signed int)IMAQdxErrorCameraMulticastNotAvailable) {  strcpy(aMessage,    "Unable to configure the system for multicast support."); return; }
    if( aError == (signed int)IMAQdxErrorBufferHasLostPackets) {  strcpy(aMessage,           "The requested buffer has lost packets and the user requested an error to be generated."); return; }
    if( aError == (signed int)IMAQdxErrorGiGEVisionError) {  strcpy(aMessage,                "Unknown GiGE Vision error."); return; }
    if( aError == (signed int)IMAQdxErrorNetworkError) {  strcpy(aMessage,                   "Unknown network error."); return; }
    if( aError == (signed int)IMAQdxErrorCameraUnreachable) {  strcpy(aMessage,              "Unable to connect to the camera."); return; }
    if( aError == (signed int)IMAQdxErrorHighPerformanceNotSupported) {  strcpy(aMessage,    "High performance acquisition is not supported on the specified network interface. Connect the camera to a network interface running the high performance driver."); return; }
    if( aError == (signed int)IMAQdxErrorInterfaceNotRenamed) {  strcpy(aMessage,            "Unable to rename interface. Invalid or duplicate name specified."); return; }
    if( aError == (signed int)IMAQdxErrorNoSupportedVideoModes) {  strcpy(aMessage,          "The camera does not have any video modes which are supported."); return; }
    if( aError == (signed int)IMAQdxErrorSoftwareTriggerOverrun) {  strcpy(aMessage,         "Software trigger overrun."); return; }
    if( aError == (signed int)IMAQdxErrorTestPacketNotReceived) {  strcpy(aMessage,          "The system did not receive a test packet from the camera. The packet size may be too large for the network configuration or a firewall may be enabled."); return; }
    if( aError == (signed int)IMAQdxErrorCorruptedImageReceived) {  strcpy(aMessage,         "The camera { ed a corrupted image."); return; }
    if( aError == (signed int)IMAQdxErrorCameraConfigurationHasChanged) {  strcpy(aMessage,  "The camera did not {  strcpy(aMessage, an image of the correct type it was configured for previously."); return; }
    if( aError == (signed int)IMAQdxErrorCameraInvalidAuthentication) {  strcpy(aMessage,    "The camera is configured with password authentication and either the user name and password were not configured or they are incorrect."); return; }
    if( aError == (signed int)IMAQdxErrorUnknownHTTPError) {  strcpy(aMessage,               "The camera { ed an unknown HTTP error."); return; }
    if( aError == (signed int)IMAQdxErrorKernelDriverUnavailable) {  strcpy(aMessage,        "Unable to attach to the kernel mode driver."); return; }
    if( aError == (signed int)IMAQdxErrorPixelFormatDecoderUnavailable) {  strcpy(aMessage,  "No decoder available for selected pixel format."); return; }
    if( aError == (signed int)IMAQdxErrorFirmwareUpdateNeeded) {  strcpy(aMessage,           "The acquisition hardware needs a firmware update before it can be used."); return; }
    if( aError == (signed int)IMAQdxErrorFirmwareUpdateRebootNeeded) {  strcpy(aMessage,     "The firmware on the acquisition hardware has been updated and the system must be rebooted before use."); return; }
    if( aError == (signed int)IMAQdxErrorLightingCurrentOutOfRange) {  strcpy(aMessage,      "The requested current level from the lighting controller is not possible."); return; }
    if( aError == (signed int)IMAQdxErrorUSB3VisionError) {  strcpy(aMessage,                "Unknown USB3 Vision error."); return; }
    if( aError == (signed int)IMAQdxErrorInvalidU3VUSBDescriptor) {  strcpy(aMessage,        "The camera has a USB descriptor that is incompatible with the USB3 Vision specification."); return; }
    if( aError == (signed int)IMAQdxErrorU3VInvalidControlInterface) {  strcpy(aMessage,     "The USB3 Vision control interface is not implemented or is invalid on this camera."); return; }
    if( aError == (signed int)IMAQdxErrorU3VControlInterfaceError) {  strcpy(aMessage,       "There was an error from the control interface of the USB3 Vision camera."); return; }
    if( aError == (signed int)IMAQdxErrorU3VInvalidEventInterface) {  strcpy(aMessage,       "The USB3 Vision event interface is not implemented or is invalid on this camera."); return; }
    if( aError == (signed int)IMAQdxErrorU3VEventInterfaceError) {  strcpy(aMessage,         "There was an error from the event interface of the USB3 Vision camera."); return; }
    if( aError == (signed int)IMAQdxErrorU3VInvalidStreamInterface) {  strcpy(aMessage,      "The USB3 Vision stream interface is not implemented or is invalid on this camera."); return; }
    if( aError == (signed int)IMAQdxErrorU3VStreamInterfaceError) {  strcpy(aMessage,        "There was an error from the stream interface of the USB3 Vision camera."); return; }
    if( aError == (signed int)IMAQdxErrorU3VUnsupportedConnectionSpeed) {  strcpy(aMessage,  "The USB connection speed is not supported by the camera.  Check whether the camera is plugged into a USB 2.0 port instead of a USB 3.0 port.  If so, {  strcpy(aMessage,  verify that the camera supports this use case."); return; }
    if( aError == (signed int)IMAQdxErrorU3VInsufficientPower) {  strcpy(aMessage,           "The USB3 Vision camera requires more current than can be supplied by the USB port in use."); return; }
    if( aError == (signed int)IMAQdxErrorU3VInvalidMaxCurrent) {  strcpy(aMessage,           "The U3V_MaximumCurrentUSB20_mA registry value is not valid for the connected USB3 Vision camera."); return; }
    if( aError == (signed int)IMAQdxErrorBufferIncompleteData) {  strcpy(aMessage,           "The requested buffer has incomplete data and the user requested an error to be generated."); return; }
    if( aError == (signed int)IMAQdxErrorCameraAcquisitionConfigFailed) {  strcpy(aMessage,  "The camera { ed an error starting the acquisition."); return; }
    if( aError == (signed int)IMAQdxErrorCameraClosePending) {  strcpy(aMessage,             "The camera still has outstanding references and will be closed when these operations complete."); return; }
    if( aError == (signed int)IMAQdxErrorSoftwareFault) {  strcpy(aMessage,                  "An unexpected software error occurred."); return; }
    if( aError == (signed int)IMAQdxErrorCameraPropertyInvalid) {  strcpy(aMessage,          "The value for an invalid camera property was requested."); return; }
    if( aError == (signed int)IMAQdxErrorJumboFramesNotEnabled) {  strcpy(aMessage,          "Jumbo frames are not enabled on the host.  Maximum packet size is 1500 bytes."); return; }
    if( aError == (signed int)IMAQdxErrorBayerPixelFormatNotSelected) {  strcpy(aMessage,    "This operation requires that the camera has a Bayer pixel format selected."); return; }
    strcpy(aMessage, "Failure but no error message.");
    return;
}

#endif // ___imaqerr_h___
