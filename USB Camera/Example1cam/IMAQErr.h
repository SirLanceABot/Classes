#ifndef ___imaqerr_h___
#define ___imaqerr_h___
#include "NIIMAQdx.h"

char * IMAQErrorMessage (int aError)
{
    if( aError == IMAQdxErrorSuccess) return                        "Success";
    if( aError == IMAQdxErrorSystemMemoryFull) return               "Not enough memory.";
    if( aError == IMAQdxErrorInternal) return                       "Internal error.";
    if( aError == IMAQdxErrorInvalidParameter) return               "Invalid parameter.";
    if( aError == IMAQdxErrorInvalidPointer) return                 "Invalid pointer.";
    if( aError == IMAQdxErrorInvalidInterface) return               "Invalid camera session.";
    if( aError == IMAQdxErrorInvalidRegistryKey) return             "Invalid registry key.";
    if( aError == IMAQdxErrorInvalidAddress) return                 "Invalid address.";
    if( aError == IMAQdxErrorInvalidDeviceType) return              "Invalid device type.";
    if( aError == IMAQdxErrorNotImplemented) return                 "Not implemented.";
    if( aError == IMAQdxErrorCameraNotFound) return                 "Camera not found.";
    if( aError == IMAQdxErrorCameraInUse) return                    "Camera is already in use.";
    if( aError == IMAQdxErrorCameraNotInitialized) return           "Camera is not initialized.";
    if( aError == IMAQdxErrorCameraRemoved) return                  "Camera has been removed.";
    if( aError == IMAQdxErrorCameraRunning) return                  "Acquisition in progress.";
    if( aError == IMAQdxErrorCameraNotRunning) return               "No acquisition in progress.";
    if( aError == IMAQdxErrorAttributeNotSupported) return          "Attribute not supported by the camera.";
    if( aError == IMAQdxErrorAttributeNotSettable) return           "Unable to set attribute.";
    if( aError == IMAQdxErrorAttributeNotReadable) return           "Unable to get attribute.";
    if( aError == IMAQdxErrorAttributeOutOfRange) return            "Attribute value is out of range.";
    if( aError == IMAQdxErrorBufferNotAvailable) return             "Requested buffer is unavailable.";
    if( aError == IMAQdxErrorBufferListEmpty) return                "Buffer list is empty. Add one or more buffers.";
    if( aError == IMAQdxErrorBufferListLocked) return               "Buffer list is already locked. Reconfigure acquisition and try again.";
    if( aError == IMAQdxErrorBufferListNotLocked) return            "No buffer list. Reconfigure acquisition and try again.";
    if( aError == IMAQdxErrorResourcesAllocated) return             "Transfer engine resources already allocated. Reconfigure acquisition and try again.";
    if( aError == IMAQdxErrorResourcesUnavailable) return           "Insufficient transfer engine resources.";
    if( aError == IMAQdxErrorAsyncWrite) return                     "Unable to perform asynchronous register write.";
    if( aError == IMAQdxErrorAsyncRead) return                      "Unable to perform asynchronous register read.";
    if( aError == IMAQdxErrorTimeout) return                        "Timeout.";
    if( aError == IMAQdxErrorBusReset) return                       "Bus reset occurred during a transaction.";
    if( aError == IMAQdxErrorInvalidXML) return                     "Unable to load camera's XML file.";
    if( aError == IMAQdxErrorFileAccess) return                     "Unable to read/write to file.";
    if( aError == IMAQdxErrorInvalidCameraURLString) return         "Camera has malformed URL string.";
    if( aError == IMAQdxErrorInvalidCameraFile) return              "Invalid camera file.";
    if( aError == IMAQdxErrorGenICamError) return                   "Unknown Genicam error.";
    if( aError == IMAQdxErrorFormat7Parameters) return              "For format 7: The combination of speed) return  image position) return  image size) return  and color coding is incorrect.";
    if( aError == IMAQdxErrorInvalidAttributeType) return           "The attribute type is not compatible with the passed variable type.";
    if( aError == IMAQdxErrorDLLNotFound) return                    "The DLL could not be found.";
    if( aError == IMAQdxErrorFunctionNotFound) return               "The function could not be found.";
    if( aError == IMAQdxErrorLicenseNotActivated) return            "License not activated.";
    if( aError == IMAQdxErrorCameraNotConfiguredForListener) return "The camera is not configured properly to support a listener.";
    if( aError == IMAQdxErrorCameraMulticastNotAvailable) return    "Unable to configure the system for multicast support.";
    if( aError == IMAQdxErrorBufferHasLostPackets) return           "The requested buffer has lost packets and the user requested an error to be generated.";
    if( aError == IMAQdxErrorGiGEVisionError) return                "Unknown GiGE Vision error.";
    if( aError == IMAQdxErrorNetworkError) return                   "Unknown network error.";
    if( aError == IMAQdxErrorCameraUnreachable) return              "Unable to connect to the camera.";
    if( aError == IMAQdxErrorHighPerformanceNotSupported) return    "High performance acquisition is not supported on the specified network interface. Connect the camera to a network interface running the high performance driver.";
    if( aError == IMAQdxErrorInterfaceNotRenamed) return            "Unable to rename interface. Invalid or duplicate name specified.";
    if( aError == IMAQdxErrorNoSupportedVideoModes) return          "The camera does not have any video modes which are supported.";
    if( aError == IMAQdxErrorSoftwareTriggerOverrun) return         "Software trigger overrun.";
    if( aError == IMAQdxErrorTestPacketNotReceived) return          "The system did not receive a test packet from the camera. The packet size may be too large for the network configuration or a firewall may be enabled.";
    if( aError == IMAQdxErrorCorruptedImageReceived) return         "The camera returned a corrupted image.";
    if( aError == IMAQdxErrorCameraConfigurationHasChanged) return  "The camera did not return an image of the correct type it was configured for previously.";
    if( aError == IMAQdxErrorCameraInvalidAuthentication) return    "The camera is configured with password authentication and either the user name and password were not configured or they are incorrect.";
    if( aError == IMAQdxErrorUnknownHTTPError) return               "The camera returned an unknown HTTP error.";
    if( aError == IMAQdxErrorKernelDriverUnavailable) return        "Unable to attach to the kernel mode driver.";
    if( aError == IMAQdxErrorPixelFormatDecoderUnavailable) return  "No decoder available for selected pixel format.";
    if( aError == IMAQdxErrorFirmwareUpdateNeeded) return           "The acquisition hardware needs a firmware update before it can be used.";
    if( aError == IMAQdxErrorFirmwareUpdateRebootNeeded) return     "The firmware on the acquisition hardware has been updated and the system must be rebooted before use.";
    if( aError == IMAQdxErrorLightingCurrentOutOfRange) return      "The requested current level from the lighting controller is not possible.";
    if( aError == IMAQdxErrorUSB3VisionError) return                "Unknown USB3 Vision error.";
    if( aError == IMAQdxErrorInvalidU3VUSBDescriptor) return        "The camera has a USB descriptor that is incompatible with the USB3 Vision specification.";
    if( aError == IMAQdxErrorU3VInvalidControlInterface) return     "The USB3 Vision control interface is not implemented or is invalid on this camera.";
    if( aError == IMAQdxErrorU3VControlInterfaceError) return       "There was an error from the control interface of the USB3 Vision camera.";
    if( aError == IMAQdxErrorU3VInvalidEventInterface) return       "The USB3 Vision event interface is not implemented or is invalid on this camera.";
    if( aError == IMAQdxErrorU3VEventInterfaceError) return         "There was an error from the event interface of the USB3 Vision camera.";
    if( aError == IMAQdxErrorU3VInvalidStreamInterface) return      "The USB3 Vision stream interface is not implemented or is invalid on this camera.";
    if( aError == IMAQdxErrorU3VStreamInterfaceError) return        "There was an error from the stream interface of the USB3 Vision camera.";
    if( aError == IMAQdxErrorU3VUnsupportedConnectionSpeed) return  "The USB connection speed is not supported by the camera.  Check whether the camera is plugged into a USB 2.0 port instead of a USB 3.0 port.  If so, return  verify that the camera supports this use case.";       
    if( aError == IMAQdxErrorU3VInsufficientPower) return           "The USB3 Vision camera requires more current than can be supplied by the USB port in use.";
    if( aError == IMAQdxErrorU3VInvalidMaxCurrent) return           "The U3V_MaximumCurrentUSB20_mA registry value is not valid for the connected USB3 Vision camera.";
    if( aError == IMAQdxErrorBufferIncompleteData) return           "The requested buffer has incomplete data and the user requested an error to be generated.";
    if( aError == IMAQdxErrorCameraAcquisitionConfigFailed) return  "The camera returned an error starting the acquisition.";
    if( aError == IMAQdxErrorCameraClosePending) return             "The camera still has outstanding references and will be closed when these operations complete.";
    if( aError == IMAQdxErrorSoftwareFault) return                  "An unexpected software error occurred.";
    if( aError == IMAQdxErrorCameraPropertyInvalid) return          "The value for an invalid camera property was requested.";
    if( aError == IMAQdxErrorJumboFramesNotEnabled) return          "Jumbo frames are not enabled on the host.  Maximum packet size is 1500 bytes.";
    if( aError == IMAQdxErrorBayerPixelFormatNotSelected) return    "This operation requires that the camera has a Bayer pixel format selected.";
	return "Failure but no error message.";
}

#endif // ___imaqerr_h___
