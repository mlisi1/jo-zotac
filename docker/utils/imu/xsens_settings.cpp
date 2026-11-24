// Reads (and optionally sets ONE thing: the RotSensor alignment) the configuration stored in the Xsens MTi.
// Uses the XDA library built with xsens_mti_ros2_driver. Stop the Xsens driver first (the port is exclusive).
//   xsens_settings                          print the stored settings, change nothing
//   xsens_settings --set-rotsensor R P Y    write the RotSensor alignment (deg), e.g. 0 0 0 = identity
// Build/run through docker/utils/imu/xsens_settings.sh (the `xsens_settings` alias).
#include <xscontroller/xsscanner.h>
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <xstypes/xsdataidentifier.h>
#include <xstypes/xsfilterprofile.h>
#include <xstypes/xseuler.h>
#include <xstypes/xsquaternion.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <cmath>

class Journaller;
Journaller* gJournal = 0;   // XDA expects the application to define it (as the driver's main.cpp does)

static void printAlignment(XsDevice* d, XsAlignmentFrame f, const char* name)
{
	XsQuaternion q = d->alignmentRotationQuaternion(f);
	XsEuler e(q);
	std::printf("  %-10s quaternion (w x y z) %+.4f %+.4f %+.4f %+.4f  = roll %+.2f pitch %+.2f yaw %+.2f deg%s\n",
		name, q.w(), q.x(), q.y(), q.z(), e.roll(), e.pitch(), e.yaw(),
		(std::fabs(std::fabs(q.w()) - 1.0) < 1e-4) ? "  (identity)" : "  <-- NOT identity");
}

int main(int argc, char** argv)
{
	bool set = false; double r = 0, p = 0, y = 0;
	if (argc == 5 && std::strcmp(argv[1], "--set-rotsensor") == 0) {
		set = true; r = std::atof(argv[2]); p = std::atof(argv[3]); y = std::atof(argv[4]);
	} else if (argc != 1) {
		std::fprintf(stderr, "usage: %s [--set-rotsensor ROLL PITCH YAW]   (degrees)\n", argv[0]);
		return 2;
	}
	XsControl* control = XsControl::construct();
	XsPortInfo port;
	for (auto const& pi : XsScanner::scanPorts())
		if (pi.deviceId().isMti() || pi.deviceId().isMtig()) { port = pi; break; }
	if (port.empty()) { std::fprintf(stderr, "No MTi found (is the Xsens driver still running? it holds the port)\n"); control->destruct(); return 1; }
	std::printf("Found %s on %s @ %d baud\n", port.deviceId().toString().toStdString().c_str(),
		port.portName().toStdString().c_str(), XsBaud::rateToNumeric(port.baudrate()));
	if (!control->openPort(port)) { std::fprintf(stderr, "Could not open the port\n"); control->destruct(); return 1; }
	XsDevice* d = control->device(port.deviceId());
	if (!d || !d->gotoConfig()) { std::fprintf(stderr, "Could not enter config mode\n"); control->destruct(); return 1; }

	std::printf("Product %s, firmware %s\n", d->productCode().toStdString().c_str(), d->firmwareVersion().toString().toStdString().c_str());
	std::printf("Filter profile: %s\n", d->onboardFilterProfile().toString().toStdString().c_str());
	XsDeviceOptionFlag f = d->deviceOptionFlags();
	std::printf("Option flags 0x%08x:%s%s%s%s%s\n", (unsigned)f,
		(f & XDOF_EnableAhs) ? " AHS" : "", (f & XDOF_EnableInrunCompassCalibration) ? " in-run-compass-cal" : "",
		(f & XDOF_EnableOrientationSmoother) ? " orientation-smoother" : "",
		(f & XDOF_EnableContinuousZRU) ? " continuous-ZRU" : "", (f & XDOF_EnableBeidou) ? " beidou" : "");
	std::printf("Output configuration (what the device sends, and at what rate):\n");
	for (auto const& oc : d->outputConfiguration())
		std::printf("  0x%04x  %4d Hz\n", (unsigned)oc.m_dataIdentifier, (int)oc.m_frequency);
	std::printf("Alignment rotations stored in the device:\n");
	printAlignment(d, XAF_Sensor, "RotSensor");
	printAlignment(d, XAF_Local, "RotLocal");

	int rc = 0;
	if (set) {
		XsQuaternion q(XsEuler(r, p, y)); q.normalize();
		std::printf("\nWriting RotSensor = roll %.2f pitch %.2f yaw %.2f deg ...\n", r, p, y);
		if (!d->setAlignmentRotationQuaternion(XAF_Sensor, q)) { std::fprintf(stderr, "FAILED to write RotSensor\n"); rc = 1; }
		else { std::printf("Written. Read back:\n"); printAlignment(d, XAF_Sensor, "RotSensor"); }
	}
	d->gotoMeasurement();
	control->closePort(port);
	control->destruct();
	return rc;
}
