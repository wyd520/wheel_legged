//
// Created by yeyinong on 2023/5/6.
//

#ifndef BIPEDALNUC_MTI_RECEIVE_DATA_H
#define BIPEDALNUC_MTI_RECEIVE_DATA_H

#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xscontroller/xsscanner.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <xstypes/xsdatapacket.h>
#include <xstypes/xstime.h>
#include <xscommon/xsens_mutex.h>

#include <iostream>
#include <iomanip>
#include <list>
#include <string>
#include <mutex>
#include "Eigen/Core"

using namespace std;

extern mutex mutex_imu;
extern bool joy_cmd_exit;
extern int imu_delay;

class CallbackHandler : public XsCallback
{
public:
    CallbackHandler(size_t maxBufferSize = 5);

    virtual ~CallbackHandler() throw();

    bool packetAvailable() const;

    XsDataPacket getNextPacket();

protected:
    void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) override;

private:
    mutable xsens::Mutex m_mutex;

    size_t m_maxNumberOfPacketsInBuffer;
    size_t m_numberOfPacketsInBuffer;
    list<XsDataPacket> m_packetBuffer;
};

int receiveImu(Eigen::Matrix<float, 10, 1>& test);
#endif //BIPEDALNUC_MTI_RECEIVE_DATA_H
