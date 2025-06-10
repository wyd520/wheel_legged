
//  Copyright (c) 2003-2021 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

//--------------------------------------------------------------------------------
// Public Xsens device API C++ example MTi receive data.
//--------------------------------------------------------------------------------

#include "mti_receive_data.h"
#include "time.h"


Journaller* gJournal = 0;

CallbackHandler::CallbackHandler(size_t maxBufferSize)
		: m_maxNumberOfPacketsInBuffer(maxBufferSize)
		, m_numberOfPacketsInBuffer(0)
{
}

CallbackHandler::~CallbackHandler() throw() {

}

bool CallbackHandler::packetAvailable() const {
    xsens::Lock locky(&m_mutex);
    return m_numberOfPacketsInBuffer > 0;
}

XsDataPacket CallbackHandler::getNextPacket()
{
    assert(packetAvailable());
    xsens::Lock locky(&m_mutex);
    XsDataPacket oldestPacket(m_packetBuffer.front());
    m_packetBuffer.pop_front();
    --m_numberOfPacketsInBuffer;
    return oldestPacket;
}

void CallbackHandler::onLiveDataAvailable(XsDevice*, const XsDataPacket* packet)
{
    xsens::Lock locky(&m_mutex);
    assert(packet != 0);
    while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer)
        (void)getNextPacket();

    m_packetBuffer.push_back(*packet);
    ++m_numberOfPacketsInBuffer;
    assert(m_numberOfPacketsInBuffer <= m_maxNumberOfPacketsInBuffer);
}


//--------------------------------------------------------------------------------
int receiveImu(Eigen::Matrix<float, 10, 1>& floatBaseMessage)
{
    cout << "Creating XsControl object..." << endl;
    XsControl* control = XsControl::construct();
    assert(control != 0);

    // Lambda function for error handling
    auto handleError = [=](string errorString)
    {
        joy_cmd_exit = true;
        control->destruct();
        cout << errorString << endl;
        cout << "exit:\nOpen MTi Device Failed.\nPlease restart process." << endl;
        return -1;
    };

    cout << "Scanning for devices..." << endl;
    XsPortInfoArray portInfoArray = XsScanner::scanPorts();

	// Find an MTi device
	XsPortInfo mtPort;
	for (auto const &portInfo : portInfoArray)
	{
		if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
		{
			mtPort = portInfo;
			break;
		}
	}

	if (mtPort.empty())
		return handleError("No MTi device found. Aborting.");

	cout << "Found a device with ID: " << mtPort.deviceId().toString().toStdString() << " @ port: " << mtPort.portName().toStdString() << ", baudrate: " << mtPort.baudrate() << endl;

	cout << "Opening port..." << endl;
	if (!control->openPort(mtPort.portName().toStdString(), mtPort.baudrate()))
		return handleError("Could not open port. Aborting.");

	// Get the device object
	XsDevice* device = control->device(mtPort.deviceId());
	assert(device != 0);

	cout << "Device: " << device->productCode().toStdString() << ", with ID: " << device->deviceId().toString() << " opened." << endl;

	// Create and attach callback handler to device
	CallbackHandler callback;
	device->addCallbackHandler(&callback);

	// Put the device into configuration mode before configuring the device
	cout << "Putting device into configuration mode..." << endl;
	if (!device->gotoConfig())
		return handleError("Could not put device into configuration mode. Aborting.");

	cout << "Configuring the device..." << endl;

	// Important for Public XDA!
	// Call this function if you want to record a mtb file:
	device->readEmtsAndDeviceConfiguration();

	XsOutputConfigurationArray configArray;
	configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
	configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));
/*---------------------------------------------------------------------------------------------------*/
	//原来的
	// if (device->deviceId().isGnss())
	// {
	// 	configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 0xFFFF));
    //     //user code begin
    //     configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 0xFFFF));

    //     configArray.push_back(XsOutputConfiguration(XDI_FreeAcceleration, 0xFFFF));
	// }else
	// {
	// 	return handleError("Unknown device while configuring. Aborting.");
	// }

	// 我修改的
	if (device->deviceId().isVru() || device->deviceId().isAhrs())
    {
		configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 400));
		configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 400));
        configArray.push_back(XsOutputConfiguration(XDI_FreeAcceleration, 400));
        cout << "isVru" << endl;
    }
    else
	{
		return handleError("Unknown device while configuring. Aborting.");
	}

/*---------------------------------------------------------------------------------------------------*/
	if (!device->setOutputConfiguration(configArray))
		return handleError("Could not configure MTi device. Aborting.");

	cout << "Creating a log file..." << endl;
	string logFileName = "logfile.mtb";
	if (device->createLogFile(logFileName) != XRV_OK)
		return handleError("Failed to create a log file. Aborting.");
	else
		cout << "Created a log file: " << logFileName.c_str() << endl;

	cout << "Putting device into measurement mode..." << endl;
	if (!device->gotoMeasurement())
		return handleError("Could not put device into measurement mode. Aborting.");

	cout << "Starting recording..." << endl;
	if (!device->startRecording())
		return handleError("Failed to start recording. Aborting.");

	cout << "\nImu Recording data successfully." << endl;
	cout << string(79, '-') << endl;

	int64_t startTime = XsTime::timeStampNow();
//	while (XsTime::timeStampNow() - startTime <= 100000)

    while (!joy_cmd_exit)
	{

		if (callback.packetAvailable())
		{
			cout << setw(5) << fixed << setprecision(2);

			// Retrieve a packet
			XsDataPacket packet = callback.getNextPacket();

			if (packet.containsOrientation())
			{
				XsQuaternion quaternion = packet.orientationQuaternion();

				XsEuler euler = packet.orientationEuler();

                XsVector gyr = packet.calibratedGyroscopeData();

                XsVector acc = packet.freeAcceleration();


                // float roll = euler.roll()/180*M_PI;
                // float pitch = euler.pitch()/180*M_PI;
                // float yaw = euler.yaw()/180*M_PI;

				float quaternionX = quaternion.x();
                float quaternionY = quaternion.y();
                float quaternionZ = quaternion.z();
                float quaternionW = quaternion.w();

                float gyrX = gyr[0];
                float gyrY = gyr[1];
                float gyrZ = gyr[2];

                float accX = acc[0];
                float accY = acc[1];
                float accZ = acc[2];

                floatBaseMessage << quaternionX, quaternionY, quaternionZ, quaternionW, gyrX, gyrY, gyrZ, accX, accY, accZ;
			}
		}
		XsTime::msleep(0);
	}
	cout << "\n" << string(79, '-') << "\n";
	cout << endl;

	cout << "Stopping recording..." << endl;
	if (!device->stopRecording())
		return handleError("Failed to stop recording. Aborting.");

	cout << "Closing log file..." << endl;
	if (!device->closeLogFile())
		return handleError("Failed to close log file. Aborting.");

	cout << "Closing port..." << endl;
	control->closePort(mtPort.portName().toStdString());

	cout << "Freeing XsControl object..." << endl;
	control->destruct();

	cout << "Successful exit." << endl;

	return 0;
}
